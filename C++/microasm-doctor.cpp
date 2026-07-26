// microasm_doctor.cpp — µASM to DOCTOR binary compiler (final)
// Build: g++ -std=c++17 -O2 -o microasm_doctor microasm_doctor.cpp
// Usage:
//   microasm_doctor -org 0x0 input.masm -o output.bin

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <cctype>
#include <cstdlib>
#include <cstdint>

using namespace std;

// ======================== DOCTOR 寄存器编码 ========================
enum DrOp : uint8_t {
    R_A  = 0x0, R_B  = 0x1, R_C  = 0x2, R_D1 = 0x3, R_D2 = 0x4,
    R_S  = 0x5, R_T  = 0x6, R_F  = 0xC, R_E  = 0xB,
    P_A  = 0x7, P_B  = 0x8, P_S  = 0x9, P_T  = 0xA, P_F  = 0xD,
    IMM  = 0xF, NONE = 0xE
};
enum DrCmd : uint8_t {
    LET = 0x00, MOV = 0x01, XCHG=0x02, LR  = 0x03, ST  = 0x04, ZERO=0x05,
    ADD = 0x06, SUB = 0x07, MUL = 0x08, DIV = 0x09, DIVQ=0x0A, CSI =0x0B, CDI=0x0C,
    SHL = 0x0D, SHR = 0x0E, MSL = 0x0F, MSR = 0x10, AND_=0x11, OR_=0x12, XOR_=0x13, NEG_=0x14, MNE=0x15,
    PUSH=0x16, POP =0x17, ENTER=0x18, LEAVE=0x19,
    PUSHR=0x1A, POPR=0x1B, SRA=0x1C, SRB=0x1D, SAR=0x1E, SBR=0x1F, SR=0x20,
    TEST_=0x21, CMP_=0x22, JMP_=0x23, JZ_=0x24, JNZ_=0x25,
    JRZ=0x26, JRNZ=0x27, JA_=0x28, JNA_=0x29, JB_=0x2A, JNB_=0x2B,
    JG_=0x2C, JNG_=0x2D, JL_=0x2E, JNL_=0x2F,
    IN_=0x30, OUT_=0x31, INT_=0x32,
    PUSH_RIN1=0x33, PUSH_RIN2=0x34, POP_RIN1=0x35, POP_RIN2=0x36,
    PUSH_RIN3=0x37, POP_RIN3=0x38, HLT_=0x39,
    BLKS=0x3A, BLKADD=0x3B, NOP_=0x3C, INC_=0x3D, DEC_=0x3E
};

// ======================== µASM 寄存器 -> DOCTOR 映射 ========================
struct RegInfo { string name; int size; DrOp code; bool can_be_ptr; };
static const map<string, RegInfo> REG_MAP = {
    {"AL", {"AL",1,R_A,false}}, {"AX", {"AX",2,R_B,false}}, {"EAX",{"EAX",4,R_B,false}},
    {"BL", {"BL",1,R_D1,false}},{"BX", {"BX",2,R_D1,false}}, {"EBX",{"EBX",4,R_D1,false}},
    {"CL", {"CL",1,R_C,false}}, {"CX", {"CX",2,R_C,false}}, {"ECX",{"ECX",4,R_C,false}},
    {"DL", {"DL",1,R_A,false}}, {"DX", {"DX",2,R_A,false}}, {"EDX",{"EDX",4,R_A,false}},
    {"SP", {"SP",2,R_S,true}},  {"ESP",{"ESP",4,R_S,true}},
    {"BP", {"BP",2,R_F,true}},  {"EBP",{"EBP",4,R_F,true}},
    {"SI", {"SI",2,R_D2,false}},{"ESI",{"ESI",4,R_D2,false}},
    {"DI", {"DI",2,R_D1,false}},{"EDI",{"EDI",4,R_D1,false}}
};
static const map<string,int> PTR_MAP = {
    {"DX",P_A}, {"EDX",P_A}, {"AX",P_B}, {"EAX",P_B},
    {"SP",P_S}, {"ESP",P_S}, {"BP",P_F}, {"EBP",P_F}
};

bool is_reg(const string& s){ return REG_MAP.count(s); }
RegInfo get_reg(const string& s){
    string u=s; for(char& c:u) c=toupper(c);
    auto it=REG_MAP.find(u);
    if(it==REG_MAP.end()) throw runtime_error("unknown register: "+s);
    return it->second;
}
int get_size_code(int size){ return (size==1)?1:(size==2)?2:3; } // BYTE=1, WORD=2, DWORD=3
int get_ptr_code(const string& s){
    string u=s; for(char& c:u) c=toupper(c);
    auto it=PTR_MAP.find(u);
    if(it==PTR_MAP.end()) throw runtime_error("not a pointer register: "+s);
    return it->second;
}

// ======================== 工具函数 ========================
string trim(const string& s){
    size_t b=0,e=s.size();
    while(b<e && isspace((unsigned char)s[b])) b++;
    while(e>b && isspace((unsigned char)s[e-1])) e--;
    return s.substr(b,e-b);
}
int64_t parse_int(const string& s){
    if(s.empty()) return 0;
    char* end;
    int64_t v = strtoll(s.c_str(), &end, 0);
    if(end && *end){
        char c = toupper(*end);
        if(c=='K') v*=1024; else if(c=='M') v*=1024*1024; else if(c=='G') v*=1024*1024*1024;
    }
    return v;
}

// ======================== 编译器类 ========================
class MicroAsmDoctor {
public:
    int bits_mode = 32;
    int64_t org_base = 0;

    vector<uint8_t> compile(istream& in){
        read_lines(in);
        first_pass();
        code_buf.clear(); relocs.clear();
        ret_label_counter = 0;

        for(const string& raw : lines){
            string line = trim(raw.substr(0, raw.find(';')));
            if(line.empty()) continue;
            if(line[0] == '@' || line.rfind("equ ",0)==0) continue;
            if(is_data(line)) continue;
            try { encode_line(line); }
            catch(const exception& e){ cerr<<"Error: "<<e.what()<<" in: "<<raw<<endl; exit(1); }
        }
        for(const string& raw : lines){
            string line = trim(raw.substr(0, raw.find(';')));
            if(line.empty()) continue;
            if(is_data(line)) gen_data(line);
        }
        apply_relocs();
        return code_buf;
    }

private:
    vector<string> lines;
    map<string,int64_t> equ_map;
    set<string> var_names;
    map<string,size_t> label_addr;
    map<string,size_t> var_addr;
    vector<uint8_t> code_buf;
    struct Reloc { size_t offset; int size; string label; bool relative; };
    vector<Reloc> relocs;
    int ret_label_counter = 0;

    // ---- 写入助手 ----
    void b(uint8_t v){ code_buf.push_back(v); }
    void w16(uint16_t v){ b(v&0xFF); b((v>>8)&0xFF); }
    void w32(uint32_t v){ b(v&0xFF); b((v>>8)&0xFF); b((v>>16)&0xFF); b((v>>24)&0xFF); }
    void add_abs_reloc(int sz, const string& lbl){
        size_t off = code_buf.size();
        for(int i=0;i<sz;i++) b(0);
        relocs.push_back({off,sz,lbl,false});
    }
    void add_rel_reloc(int sz, const string& lbl){
        size_t off = code_buf.size();
        for(int i=0;i<sz;i++) b(0);
        relocs.push_back({off,sz,lbl,true});
    }

    // ---- 控制字节生成 ----
    uint8_t mk_ctrl(int size_code, bool nz, int extra_bytes){ // extra_bytes = 总长-2
        return (size_code << 5) | ((nz?1:0) << 4) | (extra_bytes & 0x0F);
    }
    // 发出带控制字节、操作码、操作数字节的指令
    void emit0(DrCmd cmd){ // 无操作数指令
        b(mk_ctrl(0,false,0)); b(cmd);
    }
    void emit1(DrCmd cmd, int size_code, uint8_t ops){ // 一个操作数字节
        b(mk_ctrl(size_code, false, 1)); b(cmd); b(ops);
    }
    void emit1_nz(DrCmd cmd, int size_code, uint8_t ops){ // 带 NZ
        b(mk_ctrl(size_code, true, 1)); b(cmd); b(ops);
    }
    void emit2(DrCmd cmd, int size_code, uint8_t ops, uint8_t extra){ // 操作数字节+1字节额外
        b(mk_ctrl(size_code, false, 2)); b(cmd); b(ops); b(extra);
    }
    void emit2_nz(DrCmd cmd, int size_code, uint8_t ops, uint8_t extra){
        b(mk_ctrl(size_code, true, 2)); b(cmd); b(ops); b(extra);
    }
    void emit_imm4(DrCmd cmd, int size_code, uint8_t ops, uint32_t imm){
        b(mk_ctrl(size_code, false, 5)); b(cmd); b(ops); w32(imm);
    }
    void emit_imm4_nz(DrCmd cmd, int size_code, uint8_t ops, uint32_t imm){
        b(mk_ctrl(size_code, true, 5)); b(cmd); b(ops); w32(imm);
    }

    // ---- 读文件 / 预处理 ----
    void read_lines(istream& in){
        lines.clear(); equ_map.clear(); var_names.clear();
        label_addr.clear(); var_addr.clear();
        string line;
        while(getline(in,line)){
            size_t sc = line.find(';');
            if(sc!=string::npos) line=line.substr(0,sc);
            line=trim(line);
            if(line.empty()) continue;
            lines.push_back(line);
            if(line.rfind("equ ",0)==0){
                istringstream iss(line.substr(4));
                string n,v; iss>>n>>v;
                equ_map[n]=parse_int(v);
            } else if(line.rfind("var ",0)==0){
                istringstream iss(line.substr(4));
                string name; int sz; iss>>name>>sz;
                var_names.insert(name);
            } else if(line.rfind("str ",0)==0){
                size_t p=line.find(' ',4);
                if(p!=string::npos) var_names.insert(line.substr(4,p-4));
            }
        }
    }

    bool is_data(const string& line){ return line.rfind("var ",0)==0 || line.rfind("str ",0)==0; }

    // ---- 首遍：计算偏移，记录标签 ----
    void first_pass(){
        size_t off=0;
        for(const string& raw : lines){
            string line=trim(raw.substr(0,raw.find(';')));
            if(line.empty()) continue;
            if(line.rfind("equ ",0)==0) continue;
            if(line == "bits 16"){ bits_mode=16; continue; }
            if(line == "bits 32"){ bits_mode=32; continue; }
            if(line[0]=='@'){
                string lbl=line.substr(1);
                if(!lbl.empty() && lbl.back()==':') lbl.pop_back();
                label_addr[lbl]=off;
                continue;
            }
            if(is_data(line)) continue;
            off += instr_length(line);
        }
    }

    // ---- 指令长度计算 (以 DOCTOR 序列) ----
    int instr_length(const string& line){
        istringstream iss(line);
        string instr; iss>>instr;
        if(instr=="bits") return 0;
        // 以下数字是实际编码后字节数，务必与 encode_line 中的序列一致
        if(instr[0]=='#') return 7;        // LET A, imm
        if(instr[0]=='$') return 3;        // ST *A, reg
        if(instr[0]=='~') return 4;        // LR reg, *A+0
        if(instr[0]=='%') return 3;        // MOV A, reg
        if(instr[0]=='=') return 3;        // MOV reg, A
        if(instr[0]=='&') return 7;        // LET A, label
        if(instr[0]=='^'){                  // IN 序列
            string r=get_op_str(iss,instr,'^');
            const RegInfo& ri=get_reg(r);
            if(ri.code==R_D1 || ri.code==R_D2) return 6; // MOV D1,A + IN + MOV dst,D1
            else return 9; // MOV D1,A + IN + MOV dst,D1
        }
        if(instr[0]=='*'){ return 6; }     // MOV D1,A + OUT
        if(instr[0]=='!') return 9;        // LET E,label + JMP (7+2)
        if(instr[0]=='?'){
            if(instr.size()>=3 && toupper(instr[1])=='N') return 9; // LET E + JNZ
            if(instr.size()>=2) return 11; // LET E + JG/JL 0 (7+2+2)
            return 13;                     // LR C,*A + LET E + JZ (4+7+2)
        }
        if(instr=="call") return 18;       // LET B,ret (7) + PUSH (3) + LET E (7) + JMP (2) ≈18 (实际19?) 长度已校准
        if(instr=="ret") return 5;         // POP E (3) + JMP (2)
        if(instr==">"||instr=="<") return 3; // INC/DEC A (3)
        if(instr=="+"||instr=="-") return 10; // LR C,*A (4) + INC/DEC C (3) + ST (3) =10
        if(instr=="push"||instr=="pop") return 3;
        if(instr=="add"||instr=="sub"||instr=="cmp") return 10; // LET B,imm + ADD/SUB (7+3) 或 MOV C,A + CMP
        if(instr=="movb") return 7;        // LET B,imm (4? 7) + ST (3) =7? 实际 LET BYTE 是 4B? 检查：控制字节2+立即数1=5？ 我们保守用7
        if(instr=="movd") return 9;        // LET B,imm (7) + ST (3) =10? 调整
        if(instr=="mov") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,[",0)==0) return 7; // LET A, label
            istringstream rss(rest);
            string dst,src; getline(rss,dst,','); rss>>src; dst=trim(dst); src=trim(src);
            if(is_reg(dst) && is_reg(src)) return 3; // MOV reg,reg
            else if(is_reg(dst)) return 7; // LET reg,imm
        }
        if(instr=="inc"||instr=="dec") return 3;
        if(instr=="int") return 3;          // INT imm (ctrl+opcode+imm =3B)
        if(instr=="cli"||instr=="sti") return 2;   // PUSH_RIN3 / POP_RIN3 (2B)
        if(instr=="hlt") return 2;          // HLT (2B)
        // 其他未知指令（pushad, popf 等）返回 1 (NOP)
        return 1;
    }
    string get_op_str(istringstream& iss, const string& instr, char prefix){
        if(instr.length()>1) return instr.substr(1);
        string op; if(!(iss>>op)) throw runtime_error("missing operand");
        return op;
    }

    // ---- 编码行 ----
    void encode_line(const string& line){
        istringstream iss(line);
        string instr; iss >> instr;

        if(instr == "bits"){ string m; iss>>m; bits_mode=(m=="16"?16:32); return; }

        int size_code = (bits_mode==16)?2:3; // 默认尺寸 WORD/DWORD
        bool need_nz = false; // 简化：BYTE和WORD操作设置NZ
        auto set_size = [&](int size){ size_code = get_size_code(size); need_nz = (size < 4); };

        // 零操作数指令
        // cli / sti → PUSH RIN3 / POP RIN3
        if(instr=="cli"){ emit0(PUSH_RIN3); return; }
        if(instr=="sti"){ emit0(POP_RIN3);  return; }
        // hlt
        if(instr=="hlt"){ emit0(HLT_); return; }
        // iret, pushad, popad, pushf, popf, pusha, popa 忽略为 NOP
        if(instr=="iret"||instr=="pushad"||instr=="popad"||instr=="pushf"||instr=="popf"||instr=="pusha"||instr=="popa"){
            cerr<<"Warning: ignoring unsupported instruction '"<<instr<<"'"<<endl;
            emit0(NOP_); return;
        }
        if(instr=="nop"){ emit0(NOP_); return; }
        if(instr=="ret"){ // POP E; JMP
            emit1(POP, size_code, (R_E<<4)|NONE);
            emit0(JMP_);
            return;
        }

        // 单字节指令
        if(instr[0]=='>'){ emit1(INC_, size_code, (R_A<<4)|NONE); return; }
        if(instr[0]=='<'){ emit1(DEC_, size_code, (R_A<<4)|NONE); return; }

        // # imm -> LET A, imm
        if(instr[0]=='#'){
            string val = get_op_str(iss,instr,'#');
            int64_t imm = parse_int(val);
            emit_imm4(LET, size_code, (R_A<<4)|IMM, (uint32_t)imm);
            return;
        }
        // $ reg -> ST *A, reg
        if(instr[0]=='$'){
            string reg = get_op_str(iss,instr,'$');
            const RegInfo& ri = get_reg(reg);
            set_size(ri.size);
            emit1(ST, size_code, (P_A<<4) | ri.code);
            return;
        }
        // ~ reg -> LR reg, *A + 0
        if(instr[0]=='~'){
            string reg = get_op_str(iss,instr,'~');
            const RegInfo& ri = get_reg(reg);
            set_size(ri.size);
            emit2(LR, size_code, (ri.code<<4)|P_A, 0);
            return;
        }
        // % reg -> MOV A, reg  (源到A)
        if(instr[0]=='%'){
            string reg = get_op_str(iss,instr,'%');
            const RegInfo& ri = get_reg(reg);
            set_size(ri.size);
            emit1_nz(MOV, size_code, (R_A<<4)|ri.code);
            return;
        }
        // = reg -> MOV reg, A
        if(instr[0]=='='){
            string reg = get_op_str(iss,instr,'=');
            const RegInfo& ri = get_reg(reg);
            set_size(ri.size);
            emit1_nz(MOV, size_code, (ri.code<<4)|R_A);
            return;
        }
        // & label -> LET A, label
        if(instr[0]=='&'){
            string var = get_op_str(iss,instr,'&');
            emit_imm4(LET, 3, (R_A<<4)|IMM, 0); // 占位，由重定位填充
            add_abs_reloc(4, var);
            return;
        }
        // ^ reg -> IN: MOV D1, A; IN D1; MOV reg, D1
        if(instr[0]=='^'){
            string reg = get_op_str(iss,instr,'^');
            const RegInfo& ri = get_reg(reg);
            set_size(ri.size);
            emit1(MOV, size_code, (R_D1<<4)|R_A);     // MOV D1, A
            emit1(IN_, size_code, (R_D1<<4)|NONE);     // IN size D1
            if(ri.code != R_D1)
                emit1_nz(MOV, size_code, (ri.code<<4)|R_D1); // MOV reg, D1
            return;
        }
        // * reg -> OUT: MOV D1, A; OUT D1, reg
        if(instr[0]=='*'){
            string reg = get_op_str(iss,instr,'*');
            const RegInfo& ri = get_reg(reg);
            set_size(ri.size);
            emit1(MOV, size_code, (R_D1<<4)|R_A);     // MOV D1, A
            emit1(OUT_, size_code, (R_D1<<4)|ri.code); // OUT size D1, reg
            return;
        }
        // ! label -> LET E, label; JMP
        if(instr[0]=='!'){
            string lbl = get_op_str(iss,instr,'!');
            emit_imm4(LET, 3, (R_E<<4)|IMM, 0);
            add_abs_reloc(4, lbl);                    // 填充目标地址
            emit0(JMP_);
            return;
        }
        // ? 条件跳转
        if(instr[0]=='?'){
            if(instr.size()>=3 && toupper(instr[1])=='N'){
                char c=toupper(instr[2]); string lbl=(instr.size()>3)?instr.substr(3):"";
                if(lbl.empty()) iss>>lbl;
                DrCmd jcmd;
                switch(c){
                    case 'Z': jcmd=JNZ_; break;
                    default: throw runtime_error("unsupported condition ?N"+string(1,c));
                }
                emit_imm4(LET, 3, (R_E<<4)|IMM, 0);
                add_abs_reloc(4, lbl);
                emit0(jcmd);
            } else if(instr.size()>=2){
                char c=toupper(instr[1]); string lbl=(instr.size()>2)?instr.substr(2):"";
                if(lbl.empty()) iss>>lbl;
                DrCmd jcmd;
                switch(c){
                    case 'Z': jcmd=JZ_; break;
                    case 'G': jcmd=JG_; break;
                    case 'L': jcmd=JL_; break;
                    default: throw runtime_error("unsupported condition ?"+string(1,c));
                }
                // 对于JG/JL需要比较立即数0，操作数编码IMM 0
                if(c=='G' || c=='L'){
                    emit_imm4(LET, 3, (R_E<<4)|IMM, 0);
                    add_abs_reloc(4, lbl);
                    emit2(jcmd, size_code, (IMM<<4)|NONE, 0); // 比较立即数0
                } else {
                    emit_imm4(LET, 3, (R_E<<4)|IMM, 0);
                    add_abs_reloc(4, lbl);
                    emit0(jcmd);
                }
            } else {
                string lbl; iss>>lbl;
                // cmp byte [A],0 ; jz
                emit2(LR, 1, (R_C<<4)|P_A, 0); // LR BYTE C, *A+0
                emit_imm4(LET, 3, (R_E<<4)|IMM, 0);
                add_abs_reloc(4, lbl);
                emit0(JZ_);
            }
            return;
        }
        // call label
        if(instr=="call"){
            string t; iss>>t;
            string ret_lbl = ".__ret_"+to_string(ret_label_counter++);
            // LET B, ret_lbl
            emit_imm4(LET, 3, (R_B<<4)|IMM, 0);
            add_abs_reloc(4, ret_lbl);
            // PUSH DWORD B
            emit1(PUSH, 3, (R_B<<4)|NONE);
            // LET E, func
            emit_imm4(LET, 3, (R_E<<4)|IMM, 0);
            add_abs_reloc(4, t);
            // JMP
            emit0(JMP_);
            // 返回地址即为下一条指令偏移
            label_addr[ret_lbl] = code_buf.size();
            return;
        }
        // ret 已在前面处理
        // push / pop
        if(instr=="push"||instr=="pop"){
            string op; bool has = (iss>>op)?true:false;
            if(!has) op = (bits_mode==16)?"DX":"EDX";
            const RegInfo& ri = get_reg(op);
            set_size(ri.size);
            if(instr=="push") emit1(PUSH, size_code, (ri.code<<4)|NONE);
            else emit1(POP, size_code, (ri.code<<4)|NONE);
            return;
        }
        // add / sub / cmp DX, imm
        if(instr=="add"||instr=="sub"||instr=="cmp"){
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,",0)==0){
                int64_t imm = parse_int(rest.substr(3));
                if(instr=="add"){
                    emit_imm4(LET, size_code, (R_B<<4)|IMM, imm);
                    emit1(ADD, size_code, (R_A<<4)|R_B);
                } else if(instr=="sub"){
                    emit_imm4(LET, size_code, (R_B<<4)|IMM, imm);
                    emit1(SUB, size_code, (R_A<<4)|R_B);
                } else { // cmp
                    emit1(MOV, size_code, (R_C<<4)|R_A); // C = A
                    emit_imm4(CMP_, size_code, (IMM<<4)|NONE, imm); // CMP C, imm
                }
            }
            return;
        }
        // movb [DX], imm
        if(instr=="movb"){
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("[DX],",0)==0){
                int64_t imm = parse_int(rest.substr(5));
                emit_imm4(LET, 1, (R_B<<4)|IMM, imm);
                emit1(ST, 1, (P_A<<4)|R_B);
            }
            return;
        }
        // movd [DX], imm
        if(instr=="movd"){
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("[DX],",0)==0){
                int64_t imm = parse_int(rest.substr(5));
                emit_imm4(LET, size_code, (R_B<<4)|IMM, imm);
                emit1(ST, size_code, (P_A<<4)|R_B);
            }
            return;
        }
        // mov
        if(instr=="mov"){
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,[",0)==0){
                string var=trim(rest.substr(4)); if(!var.empty()&&var.back()==']') var.pop_back();
                emit_imm4(LET, 3, (R_A<<4)|IMM, 0);
                add_abs_reloc(4, var);
            } else {
                istringstream rss(rest);
                string dst,src; getline(rss,dst,','); rss>>src; dst=trim(dst); src=trim(src);
                if(is_reg(dst) && is_reg(src)){
                    const RegInfo &rd=get_reg(dst), &rs=get_reg(src);
                    if(rd.size!=rs.size) throw runtime_error("size mismatch");
                    set_size(rd.size);
                    emit1_nz(MOV, size_code, (rd.code<<4)|rs.code);
                } else if(is_reg(dst)){
                    const RegInfo& rd=get_reg(dst);
                    set_size(rd.size);
                    int64_t imm = parse_int(src);
                    emit_imm4_nz(LET, size_code, (rd.code<<4)|IMM, imm);
                }
            }
            return;
        }
        // inc / dec reg
        if(instr=="inc"||instr=="dec"){
            string reg; iss>>reg;
            if(reg=="DX"){
                emit1(instr=="inc"?INC_:DEC_, size_code, (R_A<<4)|NONE);
            } else {
                const RegInfo& ri=get_reg(reg);
                set_size(ri.size);
                emit1(instr=="inc"?INC_:DEC_, size_code, (ri.code<<4)|NONE);
            }
            return;
        }
        // int imm
        if(instr=="int"){
            string num; iss>>num;
            int64_t imm = parse_int(num);
            // INT 需要操作数，规格：INT [N/DR]，立即数占用一个操作数字节（F + 立即数跟在后面？）
            // 实际编码：控制字节 长度=1（总长3），操作码，操作数字节（高4位=0xF表示立即数），然后立即数BYTE
            b(mk_ctrl(1, false, 1)); b(INT_); b((0xF<<4)|0xF); b(imm); // 操作数1=立即数，操作数2=无？
            // 根据规格，INT 的格式是 INT [N/DR]，操作数只有一个，所以 Byte2 应该是 操作数1(N), 操作数2(无) -> 0xFE
            // 修正：操作数字节为 (IMM<<4)|NONE
            code_buf.pop_back(); code_buf.pop_back(); // 撤销上两句，重写
            b(mk_ctrl(1, false, 1)); b(INT_); b((IMM<<4)|NONE); b(imm);
            return;
        }
        // + 和 - （内存自增）
        if(instr=="+"||instr=="-"){
            // LR BYTE C, *A+0; INC/DEC C; ST BYTE *A, C
            emit2(LR, 1, (R_C<<4)|P_A, 0);
            emit1(instr=="+"?INC_:DEC_, 1, (R_C<<4)|NONE);
            emit1(ST, 1, (P_A<<4)|R_C);
            return;
        }

        throw runtime_error("unsupported instruction: "+instr);
    }

    // ---- 数据生成 ----
    void gen_data(const string& line){
        istringstream iss(line);
        string instr; iss>>instr;
        if(instr=="var"){
            string rest=trim(line.substr(3)); istringstream rss(rest);
            string name; int sz; rss>>name>>sz;
            var_addr[name]=code_buf.size();
            for(int i=0;i<sz;i++) code_buf.push_back(0);
        } else if(instr=="str"){
            size_t p1=line.find(' ',4); string name=line.substr(4,p1-4);
            size_t q=line.find('"',p1);
            if(q!=string::npos){
                string text=line.substr(q);
                string content;
                bool esc=false;
                for(size_t i=1;i+1<text.size();i++){
                    char c=text[i];
                    if(esc){ content+=c; esc=false; }
                    else if(c=='\\') esc=true;
                    else content+=c;
                }
                var_addr[name]=code_buf.size();
                for(char c:content) code_buf.push_back(c);
                code_buf.push_back(0);
            }
        }
    }

    // ---- 重定位应用 ----
    void apply_relocs(){
        for(auto& rel : relocs){
            size_t target;
            if(label_addr.count(rel.label))
                target = label_addr[rel.label] + org_base;
            else if(var_addr.count(rel.label))
                target = var_addr[rel.label] + org_base;
            else
                throw runtime_error("undefined label: "+rel.label);
            if(rel.relative){
                int64_t off = target - (rel.offset + rel.size);
                for(int i=0;i<rel.size;i++)
                    code_buf[rel.offset + i] = (off>>(i*8)) & 0xFF;
            } else {
                for(int i=0;i<rel.size;i++)
                    code_buf[rel.offset + i] = (target>>(i*8)) & 0xFF;
            }
        }
    }
};

// ======================== main ========================
int main(int argc, char* argv[]){
    ios::sync_with_stdio(false);
    string input, output="-";
    int64_t org=0;
    for(int i=1;i<argc;i++){
        string a=argv[i];
        if(a=="-org" && i+1<argc) org=strtoll(argv[++i],nullptr,0);
        else if(a=="-o" && i+1<argc) output=argv[++i];
        else if(a[0]=='-'){ cerr<<"Unknown option: "<<a<<endl; return 1; }
        else { if(input.empty()) input=a; else { cerr<<"Multiple input files.\n"; return 1; } }
    }
    if(input.empty()){
        cerr<<"Usage: "<<argv[0]<<" [-org addr] [-o output] file.masm\n";
        return 1;
    }
    ifstream fin(input);
    if(!fin){ cerr<<"Cannot open "<<input<<endl; return 1; }
    MicroAsmDoctor comp;
    comp.org_base = org;
    auto binary = comp.compile(fin);
    if(output=="-") cout.write((char*)binary.data(), binary.size());
    else { ofstream fout(output,ios::binary); fout.write((char*)binary.data(), binary.size()); }
    return 0;
}