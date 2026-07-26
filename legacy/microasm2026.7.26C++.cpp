// microasm.cpp - µASM v2 Compiler (NASM + Binary)
// g++ -std=c++17 -O2 -o microasm microasm.cpp
// Usage:
//   microasm input.masm                      -> NASM to stdout
//   microasm -o output.asm input.masm        -> NASM to file
//   microasm -bin -org 0x7C00 input.masm     -> binary to stdout
//   microasm -bin -o boot.bin input.masm     -> binary file

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <cstdint>

using namespace std;

/* ---------- 寄存器信息 ---------- */
struct RegInfo {
    string name;   // 小写 NASM 名
    int size;      // 1,2,4 字节
    int idx;       // 寄存器编码 0-7
};

static const map<string, RegInfo> REGISTERS = {
    {"AL",{"al",1,0}},{"CL",{"cl",1,1}},{"DL",{"dl",1,2}},{"BL",{"bl",1,3}},
    {"AH",{"ah",1,4}},{"CH",{"ch",1,5}},{"DH",{"dh",1,6}},{"BH",{"bh",1,7}},
    {"AX",{"ax",2,0}},{"CX",{"cx",2,1}},{"DX",{"dx",2,2}},{"BX",{"bx",2,3}},
    {"SP",{"sp",2,4}},{"BP",{"bp",2,5}},{"SI",{"si",2,6}},{"DI",{"di",2,7}},
    {"EAX",{"eax",4,0}},{"ECX",{"ecx",4,1}},{"EDX",{"edx",4,2}},{"EBX",{"ebx",4,3}},
    {"ESP",{"esp",4,4}},{"EBP",{"ebp",4,5}},{"ESI",{"esi",4,6}},{"EDI",{"edi",4,7}},
    // 段寄存器（用于识别，但编码单独处理）
    {"CS",{"cs",2,1}},{"DS",{"ds",2,3}},{"ES",{"es",2,0}},{"SS",{"ss",2,2}},
    {"FS",{"fs",2,4}},{"GS",{"gs",2,5}}
};

static const map<string,int> SEG_IDX = {{"es",0},{"cs",1},{"ss",2},{"ds",3},{"fs",4},{"gs",5}};

bool is_reg(const string& s) {
    string u = s; for(char& c:u) c=toupper(c);
    return REGISTERS.count(u);
}
const RegInfo& get_reg(const string& s) {
    string u = s; for(char& c:u) c=toupper(c);
    return REGISTERS.at(u);
}
bool is_seg(const string& s) {
    string l = s; for(char& c:l) c=tolower(c);
    return SEG_IDX.count(l);
}
int seg_idx(const string& s) {
    string l = s; for(char& c:l) c=tolower(c);
    return SEG_IDX.at(l);
}

/* ---------- 工具函数 ---------- */
string trim(const string& s) {
    size_t b=0, e=s.size();
    while(b<e && isspace((unsigned char)s[b])) b++;
    while(e>b && isspace((unsigned char)s[e-1])) e--;
    return s.substr(b, e-b);
}
int64_t parse_int(const string& s) {
    if(s.empty()) return 0;
    char* end;
    int64_t v = strtoll(s.c_str(), &end, 0);
    if(end && *end){
        char c = toupper(*end);
        if(c=='K') v*=1024; else if(c=='M') v*=1024*1024; else if(c=='G') v*=1024*1024*1024;
    }
    return v;
}

/* ---------- 表达式求值（支持 $ 和 $$） ---------- */
class ExprEval {
    const string& s;
    size_t pos;
    size_t cur_addr, base_addr;
public:
    ExprEval(const string& str, size_t cur, size_t base): s(str), pos(0), cur_addr(cur), base_addr(base) {}
    int64_t eval() {
        int64_t val = parse_add();
        if(pos != s.length()) throw runtime_error("extra characters in expression");
        return val;
    }
private:
    char peek() { return pos<s.length() ? s[pos] : 0; }
    char get() { return s[pos++]; }
    void skip_spaces() { while(peek()==' ') get(); }
    int64_t parse_add() {
        int64_t v = parse_mul();
        while(true) {
            skip_spaces();
            if(peek()=='+') { get(); v += parse_mul(); }
            else if(peek()=='-') { get(); v -= parse_mul(); }
            else break;
        }
        return v;
    }
    int64_t parse_mul() {
        int64_t v = parse_primary();
        while(true) {
            skip_spaces();
            if(peek()=='*') { get(); v *= parse_primary(); }
            else if(peek()=='/') { get(); int64_t d = parse_primary(); if(d==0) throw runtime_error("div by zero"); v /= d; }
            else break;
        }
        return v;
    }
    int64_t parse_primary() {
        skip_spaces();
        if(peek()=='(') {
            get(); int64_t v = parse_add(); skip_spaces();
            if(get()!=')') throw runtime_error("missing ')'");
            return v;
        }
        if(peek()=='$') {
            get();
            if(peek()=='$') { get(); return base_addr; }
            return cur_addr;
        }
        // 数字
        string num;
        if(peek()=='-' || peek()=='+') num += get();
        while(isdigit(peek()) || peek()=='x' || peek()=='X' || (peek()>='a' && peek()<='f') || (peek()>='A' && peek()<='F'))
            num += get();
        if(num.empty()) throw runtime_error("expected number");
        return strtoll(num.c_str(), nullptr, 0);
    }
};

/* 辅助：获取操作数（带前缀指令） */
static string get_op_str(istringstream& iss, const string& instr, char prefix) {
    if(instr.length() > 1) return instr.substr(1);
    string op; if(!(iss >> op)) throw runtime_error("missing operand");
    return op;
}

/* ========== 编译器类 ========== */
class MicroAsmCompiler {
public:
    MicroAsmCompiler() : bits_mode(32), org_base(0) {}

    // ---------- NASM 输出 ----------
    string compile_nasm(istream& in) {
        read_lines(in);
        output.clear();
        data_out.clear();
        output.push_back("section .text");
        output.push_back("global _start");
        output.push_back("_start:");
        for(const string& raw : lines) {
            string line = trim(raw.substr(0, raw.find(';')));
            if(line.empty()) continue;
            try { nasm_line(line); }
            catch(const exception& e) { cerr<<"NASM error: "<<e.what()<<" in: "<<raw<<endl; exit(1); }
        }
        if(!data_out.empty()) {
            output.push_back("");
            output.push_back("section .data");
            for(auto& s : data_out) output.push_back(s);
        }
        ostringstream oss;
        for(auto& s : output) oss << s << '\n';
        return oss.str();
    }

    // ---------- 二进制输出 ----------
    vector<uint8_t> compile_bin(istream& in) {
        read_lines(in);
        first_pass_binary();
        code_buf.clear(); relocs.clear();
        for(const string& raw : lines) {
            string line = trim(raw.substr(0, raw.find(';')));
            if(line.empty()) continue;
            if(line[0] == '@' || line.rfind("equ ",0)==0) continue;
            if(is_data_directive(line)) continue;
            try { encode_line(line); }
            catch(const exception& e) { cerr<<"Binary error: "<<e.what()<<" in: "<<raw<<endl; exit(1); }
        }
        for(const string& raw : lines) {
            string line = trim(raw.substr(0, raw.find(';')));
            if(line.empty()) continue;
            if(is_data_directive(line)) gen_data(line);
        }
        for(auto& rel : relocs) {
            size_t target;
            if(label_addr.count(rel.label))
                target = label_addr[rel.label] + org_base;
            else if(var_addr.count(rel.label))
                target = var_addr[rel.label] + org_base;
            else throw runtime_error("undefined label: " + rel.label);
            if(rel.relative) {
                int64_t off = target - (rel.offset + rel.size);
                for(int i=0;i<rel.size;i++)
                    code_buf[rel.offset + i] = (off>>(i*8)) & 0xFF;
            } else {
                for(int i=0;i<rel.size;i++)
                    code_buf[rel.offset + i] = (target>>(i*8)) & 0xFF;
            }
        }
        return code_buf;
    }

    int bits_mode;
    int64_t org_base;

private:
    vector<string> lines;
    map<string,int64_t> equ_map;
    set<string> var_names;
    map<string,size_t> label_addr;
    map<string,size_t> var_addr;
    vector<string> output;
    vector<string> data_out;
    vector<uint8_t> code_buf;
    struct Reloc {
        size_t offset;
        int size;
        string label;
        bool relative;
    };
    vector<Reloc> relocs;

    string ptr() const { return bits_mode==16 ? "dx" : "edx"; }
    string dsize() const { return bits_mode==16 ? "word" : "dword"; }
    int pidx() const { return 2; }
    string resolve_equ(const string& s) {
        if(equ_map.count(s)) return to_string(equ_map[s]);
        return s;
    }
    bool is_data_directive(const string& line) {
        return line.rfind("var ",0)==0 || line.rfind("str ",0)==0 ||
               line.rfind("db ",0)==0  || line.rfind("dw ",0)==0 ||
               line.rfind("dd ",0)==0  || line.rfind("times ",0)==0;
    }

    void read_lines(istream& in) {
        lines.clear(); equ_map.clear(); var_names.clear(); label_addr.clear(); var_addr.clear();
        string line;
        while(getline(in, line)) {
            size_t sc = line.find(';');
            if(sc != string::npos) line = line.substr(0, sc);
            line = trim(line);
            if(line.empty()) continue;
            lines.push_back(line);
            if(line.rfind("equ ",0)==0) {
                istringstream iss(line.substr(4));
                string n,v; iss>>n>>v;
                equ_map[n] = parse_int(v);
            } else if(line.rfind("var ",0)==0) {
                istringstream iss(line.substr(4));
                string name; int sz; iss>>name>>sz;
                var_names.insert(name);
            } else if(line.rfind("str ",0)==0) {
                size_t p = line.find(' ',4);
                if(p!=string::npos) var_names.insert(line.substr(4,p-4));
            }
        }
    }

    void first_pass_binary() {
        size_t off = 0;
        for(const string& raw : lines) {
            string line = trim(raw.substr(0, raw.find(';')));
            if(line.empty()) continue;
            if(line.rfind("equ ",0)==0) continue;
            if(line == "bits 16") { bits_mode=16; continue; }
            if(line == "bits 32") { bits_mode=32; continue; }
            if(line[0] == '@') {
                string lbl = line.substr(1);
                if(!lbl.empty() && lbl.back()==':') lbl.pop_back();
                label_addr[lbl] = off;
                continue;
            }
            if(is_data_directive(line)) continue;
            off += instr_length(line);
        }
    }

    int instr_length(const string& line) {
        istringstream iss(line);
        string instr; iss >> instr;
        if(instr=="bits") return 0;
        if(instr==">" || instr=="<") return 1;
        if(instr=="+" || instr=="-") return (bits_mode==16?3:2);
        if(instr=="cli"||instr=="sti"||instr=="hlt"||instr=="iret"||instr=="pushad"||instr=="popad"
           ||instr=="pushf"||instr=="popf"||instr=="pusha"||instr=="popa"||instr=="ret"||instr=="nop")
            return 1;
        if(instr=="push"||instr=="pop") {
            string op; bool has = (iss>>op)?true:false;
            if(!has) return 1;
            if(is_seg(op)) {
                string l=op; for(char& c:l) c=tolower(c);
                return (l=="fs"||l=="gs")?2:1;
            } else return 1;
        }
        if(instr[0]=='#') return 1 + (bits_mode==32?4:2);
        // $ and ~ now might have SIB (length estimation extended)
        if(instr[0]=='$' || instr[0]=='~') {
            // old form: single reg token -> 2 + optional 67h
            // new form: reg, @... -> base 2 + maybe SIB (1) + disp (1/4)
            string rest; getline(iss, rest);
            rest = trim(rest);
            size_t comma = rest.find(',');
            if(comma == string::npos) {
                return 2 + (bits_mode==16?1:0);
            } else {
                // SIB form, estimate max: 2(op+modrm) + 1(sib) + 4(disp) + prefixes
                return 2 + 1 + 4 + (bits_mode==16?1:0) + ((bits_mode==16)?0:0); // rough overestimate
            }
        }
        if(instr[0]=='=' || instr[0]=='%') {
            string r = get_op_str(iss,instr,instr[0]);
            if(is_seg(r)) return 2;  // 8C/8E + modrm
            const RegInfo& ri = get_reg(r);
            int base = 2;
            if(ri.size==2 && bits_mode==32) base=3;
            if(ri.size==4 && bits_mode==16) base=3;
            return base;
        }
        if(instr[0]=='&') return 2 + (bits_mode==32?4:2);
        if(instr[0]=='^' || instr[0]=='*') {
            string r = get_op_str(iss,instr,instr[0]);
            const RegInfo& ri = get_reg(r);
            int base=1;
            if(ri.size>1) {
                if((bits_mode==32 && ri.size==2)||(bits_mode==16 && ri.size==4)) base=2;
                else base=1;
            }
            return base;
        }
        if(instr[0]=='!') return 1 + (bits_mode==32?4:2);
        if(instr[0]=='?') {
            if(instr.size()>=3 && toupper(instr[1])=='N') return 2 + (bits_mode==32?4:2);
            if(instr.size()>=2) return 2 + (bits_mode==32?4:2);
            return 4 + (bits_mode==32?4:2);
        }
        if(instr=="call") {
            string t; iss>>t;
            if(!t.empty() && t[0]=='*') return 2;
            return 1 + (bits_mode==32?4:2);
        }
        if(instr=="add"||instr=="sub"||instr=="cmp") return 2 + (bits_mode==32?4:2);
        if(instr=="movb") return (bits_mode==16?3:2) + 1;
        if(instr=="movd") return (bits_mode==16?3:2) + (bits_mode==32?4:2);
        if(instr=="mov") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,[",0)==0) return 2 + (bits_mode==32?4:2);
            istringstream rss(rest);
            string dst,src; getline(rss,dst,','); rss>>src; dst=trim(dst); src=trim(src);
            if(is_reg(dst) && is_reg(src)) {
                const RegInfo& rd=get_reg(dst);
                int base=2;
                if(rd.size==2 && bits_mode==32) base=3;
                if(rd.size==4 && bits_mode==16) base=3;
                return base;
            } else if(is_reg(dst)) {
                const RegInfo& rd=get_reg(dst);
                if(rd.size==1) return 2;
                int base=1;
                if(bits_mode==32 && rd.size==2) base=3;
                if(bits_mode==16 && rd.size==4) base=3;
                return base + (rd.size==2?2:4);
            }
        }
        if(instr=="inc"||instr=="dec") {
            string reg; iss>>reg;
            if(reg=="DX") return 1;
            const RegInfo& ri=get_reg(reg);
            if(ri.size==2 && bits_mode==32) return 2;
            if(ri.size==4 && bits_mode==16) return 2;
            return 1;
        }
        if(instr=="int") return 2;
        if(instr=="lidt") return 3 + (bits_mode==32?4:2);
        if(instr=="out") {
            string rest; getline(iss,rest); rest=trim(rest);
            size_t c=rest.find(',');
            string r=trim(rest.substr(c+1));
            const RegInfo& ri=get_reg(r);
            int base=2;
            if((ri.size==2 && bits_mode==32)||(ri.size==4 && bits_mode==16)) base=3;
            return base;
        }
        if(instr=="in") {
            string rest; getline(iss,rest); rest=trim(rest);
            size_t c=rest.find(',');
            string r=trim(rest.substr(0,c));
            const RegInfo& ri=get_reg(r);
            int base=2;
            if((ri.size==2 && bits_mode==32)||(ri.size==4 && bits_mode==16)) base=3;
            return base;
        }
        throw runtime_error("unknown length for: " + line);
    }

    /* ---------- SIB 解析辅助 ---------- */
    struct SIBInfo {
        uint8_t base;       // base register index
        uint8_t index;      // index register index (4 if none)
        uint8_t scale;      // 0-3
        int64_t disp;       // displacement value
        bool disp_present;  // true if explicit displacement
        bool is_label;      // true if displacement is a label
        string label;       // label name if is_label
    };

    SIBInfo parse_sib(const string& expr) {
        // expr starts with '@'
        SIBInfo info;
        info.base = 0;
        info.index = 4;
        info.scale = 0;
        info.disp = 0;
        info.disp_present = false;
        info.is_label = false;

        string s = expr.substr(1); // remove '@'
        vector<string> parts;
        istringstream ss(s);
        string tok;
        while(getline(ss, tok, ',')) {
            parts.push_back(trim(tok));
        }
        if(parts.empty() || parts.size() > 4) throw runtime_error("invalid SIB expression: " + expr);
        
        // field 1: base (required)
        if(!is_reg(parts[0])) throw runtime_error("SIB base must be register: " + parts[0]);
        info.base = get_reg(parts[0]).idx;
        
        // field 2: index (optional)
        if(parts.size() >= 2) {
            string idx = parts[1];
            if(idx == "0" || idx == "1") {
                info.index = 4; // no index
            } else if(is_reg(idx)) {
                info.index = get_reg(idx).idx;
            } else {
                throw runtime_error("SIB index must be register or 0/1: " + idx);
            }
        }
        
        // field 3: scale / extra register
        bool has_second_base = false;
        uint8_t second_base_idx = 0;
        if(parts.size() >= 3) {
            string scl = parts[2];
            if(is_reg(scl)) {
                // "变成 +" mode: use scl as the index with scale 1, ignore original index?
                // Specification: Index = field3 register, Base = field1, Scale = 0
                info.index = get_reg(scl).idx;
                info.scale = 0; // x1
                has_second_base = false;
            } else {
                // numeric or #num
                if(!scl.empty() && scl[0]=='#') scl = scl.substr(1);
                int64_t scale_val = parse_int(scl);
                if(scale_val == 1) info.scale = 0;
                else if(scale_val == 2) info.scale = 1;
                else if(scale_val == 4) info.scale = 2;
                else if(scale_val == 8) info.scale = 3;
                else throw runtime_error("invalid scale: " + scl);
            }
        }
        
        // field 4: displacement
        if(parts.size() >= 4) {
            string disp_str = parts[3];
            if(disp_str == "0" || disp_str == "1") {
                info.disp_present = false;
                info.disp = 0;
            } else {
                info.disp_present = true;
                if(!disp_str.empty() && disp_str[0]=='#') disp_str = disp_str.substr(1);
                // try to parse as integer
                char* end;
                int64_t v = strtoll(disp_str.c_str(), &end, 0);
                if(end && *end) {
                    // treat as label
                    info.is_label = true;
                    info.label = disp_str;
                    info.disp = 0;
                } else {
                    info.is_label = false;
                    info.disp = v;
                }
            }
        }
        return info;
    }

    // NASM output for SIB load/store
    void nasm_sib_load_store(bool is_store, const string& sib_expr, const RegInfo& ri) {
        SIBInfo sib = parse_sib(sib_expr);
        string sz = (ri.size==1)?"byte":((ri.size==2)?"word":dsize());
        string addr = "[";
        // base
        string base_name = "eax"; // default
        for(auto& kv : REGISTERS) if(kv.second.size == 4 && kv.second.idx == sib.base) { base_name = kv.second.name; break; }
        addr += base_name;
        // index
        if(sib.index != 4) {
            string idx_name = "eax";
            for(auto& kv : REGISTERS) if(kv.second.size == 4 && kv.second.idx == sib.index) { idx_name = kv.second.name; break; }
            if(sib.scale > 0) {
                addr += "+" + idx_name + "*" + to_string(1<<sib.scale);
            } else {
                addr += "+" + idx_name;
            }
        }
        // displacement
        if(sib.is_label) {
            addr += "+" + sib.label;
        } else if(sib.disp_present) {
            if(sib.disp >= 0) addr += "+" + to_string(sib.disp);
            else addr += to_string(sib.disp); // negative sign included
        }
        addr += "]";
        if(is_store)
            output.push_back("    mov " + sz + " " + addr + ", " + ri.name);
        else
            output.push_back("    mov " + ri.name + ", " + sz + " " + addr);
    }

    // Binary encoding for SIB load/store
    void encode_sib_load_store(bool is_store, const string& sib_expr, const RegInfo& ri) {
        SIBInfo sib = parse_sib(sib_expr);
        auto b = [&](uint8_t v){ code_buf.push_back(v); };
        auto w32 = [&](uint32_t v){ b(v&0xFF); b((v>>8)&0xFF); b((v>>16)&0xFF); b((v>>24)&0xFF); };
        auto abs_reloc = [&](int sz, const string& lbl){
            size_t off=code_buf.size();
            for(int i=0;i<sz;i++) b(0);
            relocs.push_back({off,sz,lbl,false});
        };

        // Address size prefix
        if(bits_mode == 16) b(0x67);
        // Operand size prefix
        bool need_opsize = false;
        if((ri.size==2 && bits_mode==32) || (ri.size==4 && bits_mode==16))
            need_opsize = true;
        if(need_opsize) b(0x66);

        uint8_t opcode = (ri.size==1) ? (is_store ? 0x88 : 0x8A) : (is_store ? 0x89 : 0x8B);
        b(opcode);

        // Determine Mod field
        uint8_t mod = 0;
        bool use_disp8 = false;
        bool use_disp32 = false;
        int64_t disp_val = sib.disp;
        if(!sib.disp_present && !sib.is_label) {
            // no displacement
            if(sib.base == 5 && sib.index == 4) {
                // [EBP] must use disp8=0
                mod = 1;
                disp_val = 0;
                use_disp8 = true;
            } else {
                mod = 0; // no displacement
            }
        } else if(sib.is_label) {
            // label -> always disp32 (Mod=00 if base=5 and index=4, else Mod=10)
            if(sib.base == 5 && sib.index == 4) {
                mod = 0;
            } else {
                mod = 2;
            }
            use_disp32 = true;
        } else {
            // explicit displacement
            if(sib.disp >= -128 && sib.disp <= 127) {
                mod = 1;
                use_disp8 = true;
            } else {
                mod = 2;
                use_disp32 = true;
            }
        }

        uint8_t modrm = (mod << 6) | (ri.idx << 3) | 4; // r/m=4 for SIB
        b(modrm);

        uint8_t sib_byte = (sib.scale << 6) | ((sib.index & 7) << 3) | (sib.base & 7);
        b(sib_byte);

        if(use_disp8) {
            b((uint8_t)(disp_val & 0xFF));
        } else if(use_disp32) {
            if(sib.is_label) {
                abs_reloc(4, sib.label);
            } else {
                w32((uint32_t)disp_val);
            }
        }
    }

    /* ---------- NASM 行生成 ---------- */
    void nasm_line(const string& line) {
        istringstream iss(line);
        string instr; iss >> instr;
        if(instr=="bits") {
            string mode; iss >> mode;
            if(mode == "16") { bits_mode = 16; output.push_back("[bits 16]"); }
            else if(mode == "32") { bits_mode = 32; output.push_back("[bits 32]"); }
            return;
        }
        if(instr[0] == '@') {
            string lbl = instr.substr(1);
            if(!lbl.empty() && lbl.back()==':') lbl.pop_back();
            output.push_back(lbl + ":");
            return;
        }
        if(instr[0]=='#') output.push_back("    mov "+ptr()+", "+resolve_equ(get_op_str(iss,instr,'#')));
        else if(instr[0]=='$') {
            string part1, part2;
            getline(iss, part1, ',');
            getline(iss, part2);
            part1 = trim(part1);
            part2 = trim(part2);
            if(part2.empty()) {
                // old form: $ reg
                const RegInfo& ri = get_reg(part1);
                string sz = (ri.size==1)?"byte":((ri.size==2)?"word":dsize());
                output.push_back("    mov "+sz+" ["+ptr()+"], "+ri.name);
            } else {
                // new SIB form: $ reg, @...
                if(part2.empty() || part2[0]!='@') throw runtime_error("expected @SIB expression");
                const RegInfo& ri = get_reg(part1);
                nasm_sib_load_store(true, part2, ri);
            }
        }
        else if(instr[0]=='~') {
            string part1, part2;
            getline(iss, part1, ',');
            getline(iss, part2);
            part1 = trim(part1);
            part2 = trim(part2);
            if(part2.empty()) {
                const RegInfo& ri = get_reg(part1);
                string sz = (ri.size==1)?"byte":((ri.size==2)?"word":dsize());
                output.push_back("    mov "+ri.name+", "+sz+" ["+ptr()+"]");
            } else {
                if(part2.empty() || part2[0]!='@') throw runtime_error("expected @SIB expression");
                const RegInfo& ri = get_reg(part1);
                nasm_sib_load_store(false, part2, ri);
            }
        }
        else if(instr[0]=='=') {
            string r = get_op_str(iss,instr,'=');
            if(is_seg(r)) {
                string seg = r; for(char& c:seg) c=tolower(c);
                output.push_back("    mov "+seg+", "+ptr());
            } else {
                output.push_back("    mov "+get_reg(r).name+", "+ptr());
            }
        }
        else if(instr[0]=='%') {
            string r = get_op_str(iss,instr,'%');
            if(is_seg(r)) {
                string seg = r; for(char& c:seg) c=tolower(c);
                output.push_back("    mov "+ptr()+", "+seg);
            } else {
                output.push_back("    mov "+ptr()+", "+get_reg(r).name);
            }
        }
        else if(instr[0]=='&') output.push_back("    lea "+ptr()+", ["+get_op_str(iss,instr,'&')+"]");
        else if(instr[0]=='^') output.push_back("    in "+get_reg(get_op_str(iss,instr,'^')).name+", "+ptr());
        else if(instr[0]=='*') output.push_back("    out "+ptr()+", "+get_reg(get_op_str(iss,instr,'*')).name);
        else if(instr[0]=='!') output.push_back("    jmp "+get_op_str(iss,instr,'!'));
        else if(instr[0]=='?') {
            if(instr.size()>=3 && toupper(instr[1])=='N') {
                char c=toupper(instr[2]); string lbl=(instr.size()>3)?instr.substr(3):"";
                if(lbl.empty()) iss>>lbl;
                static const string ncc[]={"jnz","jnc","jno","jns","jnp"};
                int idx=string("ZCOPS").find(c);
                output.push_back("    "+ncc[idx]+" "+lbl);
            } else if(instr.size()>=2) {
                char c=toupper(instr[1]); string lbl=(instr.size()>2)?instr.substr(2):"";
                if(lbl.empty()) iss>>lbl;
                static const string jcc[]={"jz","jc","jo","js","jp","jg","jl"};
                int idx=string("ZCOPSGL").find(c);
                output.push_back("    "+jcc[idx]+" "+lbl);
            } else {
                string lbl; iss>>lbl;
                output.push_back("    cmp byte ["+ptr()+"], 0");
                output.push_back("    jz "+lbl);
            }
        }
        else if(instr=="call") {
            string t; iss>>t;
            if(!t.empty() && t[0]=='*') output.push_back("    call "+get_reg(trim(t.substr(1))).name);
            else output.push_back("    call "+resolve_equ(t));
        }
        else if(instr==">") output.push_back("    inc "+ptr());
        else if(instr=="<") output.push_back("    dec "+ptr());
        else if(instr=="+") output.push_back("    inc byte ["+ptr()+"]");
        else if(instr=="-") output.push_back("    dec byte ["+ptr()+"]");
        else if(instr=="cli") output.push_back("    cli");
        else if(instr=="sti") output.push_back("    sti");
        else if(instr=="hlt") output.push_back("    hlt");
        else if(instr=="iret") output.push_back("    iret");
        else if(instr=="pushad") output.push_back("    pushad");
        else if(instr=="popad") output.push_back("    popad");
        else if(instr=="pushf") output.push_back("    pushf");
        else if(instr=="popf")  output.push_back("    popf");
        else if(instr=="pusha") output.push_back("    pusha");
        else if(instr=="popa")  output.push_back("    popa");
        else if(instr=="ret") output.push_back("    ret");
        else if(instr=="nop") output.push_back("    nop");
        else if(instr=="push"||instr=="pop") {
            string op; bool has = (iss>>op)?true:false;
            if(instr=="push") {
                if(!has) output.push_back("    push "+ptr());
                else if(is_seg(op)) { string l=op; for(char& c:l) c=tolower(c); output.push_back("    push "+l); }
                else output.push_back("    push "+get_reg(op).name);
            } else {
                if(!has) output.push_back("    pop "+ptr());
                else if(is_seg(op)) { string l=op; for(char& c:l) c=tolower(c); output.push_back("    pop "+l); }
                else output.push_back("    pop "+get_reg(op).name);
            }
        }
        else if(instr=="add"||instr=="sub"||instr=="cmp") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,",0)==0) output.push_back("    "+instr+" "+ptr()+", "+resolve_equ(trim(rest.substr(3))));
            else throw runtime_error("unsupported "+instr);
        }
        else if(instr=="movb") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("[DX],",0)==0) output.push_back("    mov byte ["+ptr()+"], "+resolve_equ(trim(rest.substr(5))));
            else throw runtime_error("invalid movb");
        }
        else if(instr=="movd") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("[DX],",0)==0) output.push_back("    mov "+dsize()+" ["+ptr()+"], "+resolve_equ(trim(rest.substr(5))));
            else throw runtime_error("invalid movd");
        }
        else if(instr=="mov") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,[",0)==0) {
                string var=trim(rest.substr(4));
                if(!var.empty() && var.back()==']') var.pop_back();
                output.push_back("    mov "+ptr()+", ["+var+"]");
            } else {
                istringstream rss(rest);
                string dst,src; getline(rss,dst,','); rss>>src; dst=trim(dst); src=trim(src);
                if(is_reg(dst) && is_reg(src)) {
                    const RegInfo &rd=get_reg(dst), &rs=get_reg(src);
                    if(rd.size!=rs.size) throw runtime_error("size mismatch");
                    output.push_back("    mov "+rd.name+", "+rs.name);
                } else if(is_reg(dst)) {
                    output.push_back("    mov "+get_reg(dst).name+", "+resolve_equ(src));
                } else throw runtime_error("invalid mov");
            }
        }
        else if(instr=="inc"||instr=="dec") {
            string reg; iss>>reg;
            if(reg=="DX") output.push_back("    "+instr+" "+ptr());
            else output.push_back("    "+instr+" "+get_reg(reg).name);
        }
        else if(instr=="int") { string num; iss>>num; output.push_back("    int "+resolve_equ(num)); }
        else if(instr=="lidt") { string addr; iss>>addr; output.push_back("    lidt ["+addr+"]"); }
        else if(instr=="out") {
            string rest; getline(iss,rest); rest=trim(rest);
            size_t c=rest.find(','); output.push_back("    out "+resolve_equ(trim(rest.substr(0,c)))+", "+get_reg(trim(rest.substr(c+1))).name);
        }
        else if(instr=="in") {
            string rest; getline(iss,rest); rest=trim(rest);
            size_t c=rest.find(','); output.push_back("    in "+get_reg(trim(rest.substr(0,c))).name+", "+resolve_equ(trim(rest.substr(c+1))));
        }
        else if(instr=="var") {
            string rest=trim(line.substr(3)); istringstream rss(rest); string name; int sz; rss>>name>>sz;
            data_out.push_back(name+": times "+to_string(sz)+" db 0");
        }
        else if(instr=="str") {
            size_t p=line.find('"'); if(p!=string::npos) {
                string part1=line.substr(4,p-4); istringstream nss(part1); string name; nss>>name;
                string text=line.substr(p); data_out.push_back(name+": db "+text+", 0");
            }
        }
        else if(instr=="times"||instr=="db"||instr=="dw"||instr=="dd") {
            string rest=trim(line.substr(instr.size())); output.push_back("    "+instr+" "+rest);
        }
        else throw runtime_error("unknown instruction: "+instr);
    }

    /* ---------- 二进制编码 ---------- */
    void encode_line(const string& line) {
        istringstream iss(line);
        string instr; iss >> instr;

        if(instr == "bits") {
            string mode; iss >> mode;
            if(mode == "16") bits_mode = 16;
            else if(mode == "32") bits_mode = 32;
            return;
        }

        auto b = [&](uint8_t v){ code_buf.push_back(v); };
        auto w16= [&](uint16_t v){ b(v&0xFF); b((v>>8)&0xFF); };
        auto w32= [&](uint32_t v){ b(v&0xFF); b((v>>8)&0xFF); b((v>>16)&0xFF); b((v>>24)&0xFF); };
        auto abs_reloc = [&](int sz, const string& lbl){
            size_t off=code_buf.size();
            for(int i=0;i<sz;i++) b(0);
            relocs.push_back({off,sz,lbl,false});
        };
        auto rel_reloc = [&](int sz, const string& lbl){
            size_t off=code_buf.size();
            for(int i=0;i<sz;i++) b(0);
            relocs.push_back({off,sz,lbl,true});
        };

        int p = pidx();
        if(instr[0]=='#') {
            string val = get_op_str(iss,instr,'#'); int64_t imm = parse_int(resolve_equ(val));
            if(bits_mode==16){b(0xBA); w16(imm);} else{b(0xBA); w32(imm);}
        } else if(instr[0]=='$') {
            string part1, part2;
            getline(iss, part1, ',');
            getline(iss, part2);
            part1 = trim(part1);
            part2 = trim(part2);
            if(part2.empty()) {
                // old form: $ reg
                const RegInfo& ri = get_reg(part1);
                if(bits_mode==16) b(0x67); b(0x88); b((0<<6)|(ri.idx<<3)|p);
            } else {
                if(part2[0]!='@') throw runtime_error("expected @SIB expression");
                const RegInfo& ri = get_reg(part1);
                encode_sib_load_store(true, part2, ri);
            }
        } else if(instr[0]=='~') {
            string part1, part2;
            getline(iss, part1, ',');
            getline(iss, part2);
            part1 = trim(part1);
            part2 = trim(part2);
            if(part2.empty()) {
                const RegInfo& ri = get_reg(part1);
                if(bits_mode==16) b(0x67);
                if(ri.size==1) b(0x8A);
                else { if((ri.size==2&&bits_mode==32)||(ri.size==4&&bits_mode==16)) b(0x66); b(0x8B); }
                b((0<<6)|(ri.idx<<3)|p);
            } else {
                if(part2[0]!='@') throw runtime_error("expected @SIB expression");
                const RegInfo& ri = get_reg(part1);
                encode_sib_load_store(false, part2, ri);
            }
        } else if(instr[0]=='%') {
            string r = get_op_str(iss,instr,'%');
            if(is_seg(r)) {
                // mov dx, seg
                int s = seg_idx(r);
                b(0x8C);
                b((3<<6) | (s<<3) | p);
            } else {
                const RegInfo& ri = get_reg(r);
                if(ri.size==2 && bits_mode==32) b(0x66);
                else if(ri.size==4 && bits_mode==16) b(0x66);
                b(0x89);
                b((3<<6) | (ri.idx<<3) | p);
            }
        } else if(instr[0]=='=') {
            string r = get_op_str(iss,instr,'=');
            if(is_seg(r)) {
                // mov seg, dx
                int s = seg_idx(r);
                b(0x8E);
                b((3<<6) | (s<<3) | p);
            } else {
                const RegInfo& ri = get_reg(r);
                if(ri.size==2 && bits_mode==32) b(0x66);
                else if(ri.size==4 && bits_mode==16) b(0x66);
                b(0x89);
                b((3<<6) | (p<<3) | ri.idx);
            }
        } else if(instr[0]=='&') {
            string var = get_op_str(iss,instr,'&');
            if(bits_mode==32){ b(0x8D); b(0x15); abs_reloc(4,var); }
            else { b(0x8D); b(0x16); abs_reloc(2,var); }
        } else if(instr[0]=='^') {
            string r = get_op_str(iss,instr,'^'); const RegInfo& ri = get_reg(r);
            if(ri.size==1) b(0xEC); else {
                if((ri.size==2&&bits_mode==32)||(ri.size==4&&bits_mode==16)) b(0x66); b(0xED);
            }
        } else if(instr[0]=='*') {
            string r = get_op_str(iss,instr,'*'); const RegInfo& ri = get_reg(r);
            if(ri.size==1) b(0xEE); else {
                if((ri.size==2&&bits_mode==32)||(ri.size==4&&bits_mode==16)) b(0x66); b(0xEF);
            }
        } else if(instr[0]=='!') {
            string lbl = get_op_str(iss,instr,'!');
            b(0xE9); rel_reloc(bits_mode==32?4:2, lbl);
        } else if(instr[0]=='?') {
            if(instr.size()>=3 && toupper(instr[1])=='N') {
                char c=toupper(instr[2]); string lbl=(instr.size()>3)?instr.substr(3):"";
                if(lbl.empty()) iss>>lbl;
                static uint8_t ncc[256]={0};
                if(!ncc['Z']){ ncc['Z']=0x85; ncc['C']=0x83; ncc['O']=0x81; ncc['S']=0x89; ncc['P']=0x8B; }
                b(0x0F); b(ncc[(int)c]); rel_reloc(bits_mode==32?4:2, lbl);
            } else if(instr.size()>=2) {
                char c=toupper(instr[1]); string lbl=(instr.size()>2)?instr.substr(2):"";
                if(lbl.empty()) iss>>lbl;
                static uint8_t jcc[256]={0};
                if(!jcc['Z']){ jcc['Z']=0x84; jcc['C']=0x82; jcc['O']=0x80; jcc['S']=0x88; jcc['P']=0x8A; jcc['G']=0x8F; jcc['L']=0x8C; }
                b(0x0F); b(jcc[(int)c]); rel_reloc(bits_mode==32?4:2, lbl);
            } else {
                if(bits_mode==16) b(0x67); b(0x80); b(0x3A); b(0x00);
                string lbl; iss>>lbl;
                b(0x0F); b(0x84); rel_reloc(bits_mode==32?4:2, lbl);
            }
        } else if(instr=="call") {
            string t; iss>>t;
            if(!t.empty() && t[0]=='*') {
                string r=trim(t.substr(1)); const RegInfo& ri=get_reg(r);
                b(0xFF); b((3<<6)|(2<<3)|ri.idx);
            } else { b(0xE8); rel_reloc(bits_mode==32?4:2, t); }
        } else if(instr==">" || instr=="<") b(((instr==">")?0x40:0x48)+p);
        else if(instr=="+" || instr=="-") { if(bits_mode==16) b(0x67); b(0xFE); b((instr=="+")?0x02:0x0A); }
        else if(instr=="cli") b(0xFA);
        else if(instr=="sti") b(0xFB);
        else if(instr=="hlt") b(0xF4);
        else if(instr=="iret") b(0xCF);
        else if(instr=="pushad") b(0x60);
        else if(instr=="popad") b(0x61);
        else if(instr=="pushf") b(0x9C);
        else if(instr=="popf") b(0x9D);
        else if(instr=="pusha") b(0x60);
        else if(instr=="popa") b(0x61);
        else if(instr=="ret") b(0xC3);
        else if(instr=="nop") b(0x90);
        else if(instr=="push"||instr=="pop") {
            string op; bool has = (iss>>op)?true:false;
            if(instr=="push") {
                if(!has) b(0x50+p);
                else if(is_seg(op)) {
                    int s=seg_idx(op);
                    if(op=="fs"){b(0x0F);b(0xA0);} else if(op=="gs"){b(0x0F);b(0xA8);}
                    else{static uint8_t enc[]={0x06,0x0E,0x16,0x1E}; b(enc[s]);}
                } else {
                    const RegInfo& ri=get_reg(op);
                    if(ri.size==2&&bits_mode==32) b(0x66); else if(ri.size==4&&bits_mode==16) b(0x66);
                    b(0x50+ri.idx);
                }
            } else {
                if(!has) b(0x58+p);
                else if(is_seg(op)) {
                    int s=seg_idx(op);
                    if(op=="fs"){b(0x0F);b(0xA1);} else if(op=="gs"){b(0x0F);b(0xA9);}
                    else{static uint8_t enc[]={0x07,0x0F,0x17,0x1F}; b(enc[s]);}
                } else {
                    const RegInfo& ri=get_reg(op);
                    if(ri.size==2&&bits_mode==32) b(0x66); else if(ri.size==4&&bits_mode==16) b(0x66);
                    b(0x58+ri.idx);
                }
            }
        } else if(instr=="add"||instr=="sub"||instr=="cmp") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,",0)==0) {
                int64_t imm=parse_int(resolve_equ(trim(rest.substr(3))));
                uint8_t mod=(instr=="add")?0:((instr=="sub")?5:7);
                if(bits_mode==16){b(0x81); b((3<<6)|(mod<<3)|p); w16(imm);}
                else{b(0x81); b((3<<6)|(mod<<3)|p); w32(imm);}
            }
        } else if(instr=="movb") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("[DX],",0)==0) {
                int64_t imm=parse_int(resolve_equ(trim(rest.substr(5))));
                if(bits_mode==16) b(0x67); b(0xC6); b(0x02); b(imm);
            }
        } else if(instr=="movd") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("[DX],",0)==0) {
                int64_t imm=parse_int(resolve_equ(trim(rest.substr(5))));
                if(bits_mode==16) b(0x67); b(0xC7); b(0x02);
                if(bits_mode==32) w32(imm); else w16(imm);
            }
        } else if(instr=="mov") {
            string rest; getline(iss,rest); rest=trim(rest);
            if(rest.rfind("DX,[",0)==0) {
                string var=trim(rest.substr(4)); if(!var.empty()&&var.back()==']') var.pop_back();
                if(bits_mode==32){b(0x8B); b(0x15); abs_reloc(4,var);}
                else{b(0x8B); b(0x16); abs_reloc(2,var);}
            } else {
                istringstream rss(rest);
                string dst,src; getline(rss,dst,','); rss>>src; dst=trim(dst); src=trim(src);
                if(is_reg(dst) && is_reg(src)) {
                    const RegInfo &rd=get_reg(dst), &rs=get_reg(src);
                    if(rd.size!=rs.size) throw runtime_error("size mismatch");
                    if((rd.size==2&&bits_mode==32)||(rd.size==4&&bits_mode==16)) b(0x66);
                    b(rd.size==1?0x88:0x89); b((3<<6)|(rs.idx<<3)|rd.idx);
                } else if(is_reg(dst)) {
                    const RegInfo& rd=get_reg(dst);
                    int64_t imm=parse_int(resolve_equ(src));
                    if(rd.size==1){ b(0xB0+rd.idx); b(imm); }
                    else {
                        if((rd.size==2&&bits_mode==32)||(rd.size==4&&bits_mode==16)) b(0x66);
                        b(0xB8+rd.idx);
                        if(rd.size==2) w16(imm); else w32(imm);
                    }
                }
            }
        } else if(instr=="inc"||instr=="dec") {
            string reg; iss>>reg;
            if(reg=="DX") b(((instr=="inc")?0x40:0x48)+p);
            else {
                const RegInfo& ri=get_reg(reg);
                if((ri.size==2&&bits_mode==32)||(ri.size==4&&bits_mode==16)) b(0x66);
                b(((instr=="inc")?0x40:0x48)+ri.idx);
            }
        } else if(instr=="int") { string num; iss>>num; b(0xCD); b(parse_int(resolve_equ(num))); }
        else if(instr=="lidt") {
            string addr; iss>>addr;
            if(bits_mode==32){b(0x0F);b(0x01);b(0x1D);abs_reloc(4,addr);}
            else{b(0x0F);b(0x01);b(0x1E);abs_reloc(2,addr);}
        } else if(instr=="out") {
            string rest; getline(iss,rest); rest=trim(rest);
            size_t c=rest.find(','); string imm=trim(rest.substr(0,c)), r=trim(rest.substr(c+1));
            const RegInfo& ri=get_reg(r); int64_t port=parse_int(resolve_equ(imm));
            if(ri.size==1){b(0xE6);b(port);}
            else { if((ri.size==2&&bits_mode==32)||(ri.size==4&&bits_mode==16)) b(0x66); b(0xE7); b(port); }
        } else if(instr=="in") {
            string rest; getline(iss,rest); rest=trim(rest);
            size_t c=rest.find(','); string r=trim(rest.substr(0,c)), imm=trim(rest.substr(c+1));
            const RegInfo& ri=get_reg(r); int64_t port=parse_int(resolve_equ(imm));
            if(ri.size==1){b(0xE4);b(port);}
            else { if((ri.size==2&&bits_mode==32)||(ri.size==4&&bits_mode==16)) b(0x66); b(0xE5); b(port); }
        } else throw runtime_error("unsupported instruction: "+instr);
    }

    void gen_data(const string& line) {
        istringstream iss(line);
        string instr; iss >> instr;
        if(instr=="var") {
            string rest=trim(line.substr(3)); istringstream rss(rest);
            string name; int sz; rss>>name>>sz;
            var_addr[name] = code_buf.size();
            for(int i=0;i<sz;i++) code_buf.push_back(0);
        } else if(instr=="str") {
            size_t p1=line.find(' ',4); string name=line.substr(4,p1-4);
            size_t q=line.find('"',p1);
            if(q!=string::npos) {
                string text=line.substr(q);
                string content; bool esc=false;
                for(size_t i=1;i+1<text.size();i++) {
                    char c=text[i];
                    if(esc){content+=c; esc=false;}
                    else if(c=='\\') esc=true;
                    else content+=c;
                }
                var_addr[name] = code_buf.size();
                for(char c:content) code_buf.push_back(c);
                code_buf.push_back(0);
            }
        } else if(instr=="db" || instr=="dw" || instr=="dd") {
            string rest=trim(line.substr(instr.size()));
            istringstream rss(rest);
            string tok;
            while(getline(rss,tok,',')) {
                tok=trim(tok);
                if(!tok.empty() && tok[0]=='"') { for(char c:tok) if(c!='"') code_buf.push_back(c); }
                else {
                    int64_t v=parse_int(tok);
                    if(instr=="db") code_buf.push_back(v);
                    else if(instr=="dw") { code_buf.push_back(v&0xFF); code_buf.push_back((v>>8)&0xFF); }
                    else if(instr=="dd") { code_buf.push_back(v&0xFF); code_buf.push_back((v>>8)&0xFF); code_buf.push_back((v>>16)&0xFF); code_buf.push_back((v>>24)&0xFF); }
                }
            }
        } else if(instr=="times") {
            string rest=trim(line.substr(5));
            istringstream rss(rest);
            string count_expr, cmd, val;
            rss >> count_expr >> cmd;
            getline(rss, val);
            size_t cur = code_buf.size() + org_base;
            ExprEval eval(count_expr, cur, org_base);
            int64_t cnt = eval.eval();
            if(cnt < 0) throw runtime_error("times count negative");
            if(cmd=="db") {
                val=trim(val); uint8_t byte = parse_int(val);
                for(int64_t i=0;i<cnt;i++) code_buf.push_back(byte);
            } else if(cmd=="dw") {
                val=trim(val); uint16_t word = parse_int(val);
                for(int64_t i=0;i<cnt;i++) { code_buf.push_back(word&0xFF); code_buf.push_back((word>>8)&0xFF); }
            } else if(cmd=="dd") {
                val=trim(val); uint32_t dword = parse_int(val);
                for(int64_t i=0;i<cnt;i++) { code_buf.push_back(dword&0xFF); code_buf.push_back((dword>>8)&0xFF); code_buf.push_back((dword>>16)&0xFF); code_buf.push_back((dword>>24)&0xFF); }
            } else throw runtime_error("unknown times data type: "+cmd);
        }
    }
};

/* ---------- 主程序 ---------- */
int main(int argc, char* argv[]) {
    ios::sync_with_stdio(false);
    string input, output="-";
    bool bin_mode=false;
    int64_t org=0;
    for(int i=1;i<argc;i++) {
        string a=argv[i];
        if(a=="-bin") bin_mode=true;
        else if(a=="-org" && i+1<argc) org=strtoll(argv[++i],nullptr,0);
        else if(a=="-o" && i+1<argc) output=argv[++i];
        else if(a[0]=='-') { cerr<<"Unknown option: "<<a<<endl; return 1; }
        else { if(input.empty()) input=a; else { cerr<<"Multiple input files.\n"; return 1; } }
    }
    if(input.empty()) {
        cerr<<"Usage: "<<argv[0]<<" [-bin] [-org addr] [-o output] file.masm\n";
        return 1;
    }
    ifstream fin(input);
    if(!fin) { cerr<<"Cannot open "<<input<<endl; return 1; }
    MicroAsmCompiler comp;
    comp.org_base = org;
    if(bin_mode) {
        auto binary = comp.compile_bin(fin);
        if(output=="-") cout.write((char*)binary.data(), binary.size());
        else { ofstream fout(output,ios::binary); fout.write((char*)binary.data(), binary.size()); }
    } else {
        string asm_code = comp.compile_nasm(fin);
        if(output=="-") cout<<asm_code;
        else { ofstream fout(output); fout<<asm_code; }
    }
    return 0;
}