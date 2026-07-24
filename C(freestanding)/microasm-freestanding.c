/*
 * microasm.c - µASM compiler in pure C (no libc, freestanding)
 *
 * Compiles µASM source to x86 flat binary.
 * Usage: microasm_compile(source, output_buf, max_size, org_base)
 * Returns binary size, or -1 on error.
 */

/* ---- configurable types ---- */
typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
typedef signed int     s32;
typedef int            bool;
#define true  1
#define false 0
#define NULL ((void*)0)

/* ---- minimal string helpers ---- */
int my_strlen(const char *s) {
    int n = 0;
    while (*s++) n++;
    return n;
}
int my_strcmp(const char *a, const char *b) {
    while (*a && *b && *a == *b) { a++; b++; }
    return *(const unsigned char*)a - *(const unsigned char*)b;
}
int my_strncmp(const char *a, const char *b, int n) {
    while (n && *a && *b && *a == *b) { a++; b++; n--; }
    return n ? *(const unsigned char*)a - *(const unsigned char*)b : 0;
}
void my_strcpy(char *d, const char *s) {
    while ((*d++ = *s++));
}
char *my_strdup(const char *s, char *buf, int max) {
    int i = 0;
    while (i < max-1 && s[i]) { buf[i] = s[i]; i++; }
    buf[i] = 0;
    return buf;
}
int my_isspace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
int my_isdigit(char c) { return c >= '0' && c <= '9'; }
int my_isupper(int c) { return c >= 'A' && c <= 'Z'; }
int my_islower(int c) { return c >= 'a' && c <= 'z'; }
int my_toupper(int c) { return my_islower(c) ? c - 'a' + 'A' : c; }
int my_tolower(int c) { return my_isupper(c) ? c - 'A' + 'a' : c; }

/* simple string to integer (base auto-detect: hex if 0x prefix, else decimal) */
s32 my_atoi(const char *s) {
    s32 val = 0;
    int neg = 0;
    while (my_isspace(*s)) s++;
    if (*s == '-') { neg = 1; s++; }
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        s += 2;
        while (1) {
            char c = *s;
            if (c >= '0' && c <= '9') val = val * 16 + (c - '0');
            else if (c >= 'a' && c <= 'f') val = val * 16 + (c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') val = val * 16 + (c - 'A' + 10);
            else break;
            s++;
        }
    } else {
        while (my_isdigit(*s)) {
            val = val * 10 + (*s - '0');
            s++;
        }
    }
    return neg ? -val : val;
}

/* ---- tokenizer / parser ---- */
typedef struct {
    const char *p;
    char token[128];
} Scanner;

void scan_init(Scanner *s, const char *line) { s->p = line; }
int scan_next(Scanner *s) {
    while (my_isspace(*s->p)) s->p++;
    if (*s->p == 0) return 0;
    char *out = s->token;
    if (*s->p == ',') {
        *out++ = ',';
        *out = 0;
        s->p++;
        return 1;
    }
    while (*s->p && !my_isspace(*s->p) && *s->p != ',' && *s->p != ';') {
        *out++ = *s->p++;
    }
    *out = 0;
    return 1;
}

/* ---- register table (linear search, small enough) ---- */
typedef struct { const char *name; u8 size; u8 idx; } RegInfo;
#define NUM_REGS 30
static const RegInfo regs[NUM_REGS] = {
    {"al",1,0},{"cl",1,1},{"dl",1,2},{"bl",1,3},{"ah",1,4},{"ch",1,5},{"dh",1,6},{"bh",1,7},
    {"ax",2,0},{"cx",2,1},{"dx",2,2},{"bx",2,3},{"sp",2,4},{"bp",2,5},{"si",2,6},{"di",2,7},
    {"eax",4,0},{"ecx",4,1},{"edx",4,2},{"ebx",4,3},{"esp",4,4},{"ebp",4,5},{"esi",4,6},{"edi",4,7},
    {"cs",2,1},{"ds",2,3},{"es",2,0},{"ss",2,2},{"fs",2,4},{"gs",2,5}
};
int reg_find(const char *name) {
    int i;
    for (i = 0; i < NUM_REGS; i++) {
        const char *rname = regs[i].name;
        const char *p = name;
        while (*rname && *p && my_tolower(*rname) == my_tolower(*p)) { rname++; p++; }
        if (*rname == 0 && *p == 0) return i;
    }
    return -1;
}
int is_seg_reg(const char *name) {
    int idx = reg_find(name);
    return (idx >= 24 && idx < 30); /* CS..GS indices 24..29 */
}

/* ---- label / symbol table (linear, small) ---- */
#define MAX_LABELS 256
#define MAX_EQU    128
typedef struct { char name[64]; u32 addr; } LabelEntry;
static LabelEntry labels[MAX_LABELS];
static int label_cnt = 0;
static LabelEntry equs[MAX_EQU];
static int equ_cnt = 0;

u32 label_lookup(const char *name) {
    int i;
    for (i = 0; i < label_cnt; i++) {
        if (my_strcmp(labels[i].name, name) == 0) return labels[i].addr;
    }
    return 0xFFFFFFFF;
}
void label_add(const char *name, u32 addr) {
    if (label_cnt < MAX_LABELS) {
        my_strcpy(labels[label_cnt].name, name);
        labels[label_cnt].addr = addr;
        label_cnt++;
    }
}
int equ_lookup(const char *name, s32 *val) {
    int i;
    for (i = 0; i < equ_cnt; i++) {
        if (my_strcmp(equs[i].name, name) == 0) { *val = (s32)equs[i].addr; return 1; }
    }
    return 0;
}
void equ_add(const char *name, s32 val) {
    if (equ_cnt < MAX_EQU) {
        my_strcpy(equs[equ_cnt].name, name);
        equs[equ_cnt].addr = (u32)val;
        equ_cnt++;
    }
}

/* ---- expression evaluator (simplified for "times" support with $, $$) ---- */
s32 expr_eval(const char *s, u32 current_addr, u32 base_addr) {
    /* only supports integers, $, $$, +, -, *, /, and parentheses.
       We implement recursive descent, but keep it compact. */
    const char *p = s;
    char token[64];
    int token_len;
    /* next_token: parse next integer or operator */
    #define next_token() do { \
        while (my_isspace(*p)) p++; \
        token_len = 0; \
        if (*p == 0) { token[0]=0; } \
        else if (*p == '+' || *p == '-' || *p == '*' || *p == '/' || *p == '(' || *p == ')') { \
            token[0] = *p++; token[1] = 0; token_len = 1; \
        } else if ((*p >= '0' && *p <= '9') || (*p == '$')) { \
            int idx = 0; \
            while ((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F') || *p == 'x' || *p == '$') { \
                if (idx < 63) token[idx++] = *p; p++; \
            } token[idx] = 0; token_len = idx; \
        } else { token[0] = *p++; token[1]=0; token_len=1; } \
    } while(0)

    s32 val_stack[16]; int val_top = 0;
    char op_stack[16]; int op_top = 0;
    #define apply_op() do { \
        s32 b = val_stack[--val_top]; s32 a = val_stack[--val_top]; \
        char op = op_stack[--op_top]; \
        if (op == '+') val_stack[val_top++] = a + b; \
        else if (op == '-') val_stack[val_top++] = a - b; \
        else if (op == '*') val_stack[val_top++] = a * b; \
        else if (op == '/') { if (b==0) return 0; val_stack[val_top++] = a / b; } \
    } while(0)

    int precedence(char op) { return (op == '+' || op == '-') ? 1 : (op == '*' || op == '/') ? 2 : 0; }

    next_token();
    while (token[0] != 0) {
        if (token[0] == '(') {
            op_stack[op_top++] = '(';
        } else if (token[0] == ')') {
            while (op_top > 0 && op_stack[op_top-1] != '(') apply_op();
            if (op_top > 0 && op_stack[op_top-1] == '(') op_top--;
        } else if (token[0] == '+' || token[0] == '-' || token[0] == '*' || token[0] == '/') {
            while (op_top > 0 && precedence(op_stack[op_top-1]) >= precedence(token[0])) apply_op();
            op_stack[op_top++] = token[0];
        } else { /* number or $ */
            s32 num;
            if (token[0] == '$') {
                if (token[1] == '$') { num = (s32)base_addr; } else { num = (s32)current_addr; }
            } else {
                num = my_atoi(token);
            }
            val_stack[val_top++] = num;
        }
        next_token();
    }
    while (op_top > 0) apply_op();
    return val_top > 0 ? val_stack[0] : 0;
}

/* ---- compiler core ---- */
typedef struct {
    u8 *code;          /* output buffer */
    int size;          /* current length */
    int capacity;      /* max size */
    int bits_mode;     /* 16 or 32 */
    u32 org_base;
    int pass;          /* 1 = size calc, 2 = emit */
    /* relocation table */
    struct { u32 offset; int size; char label[64]; int relative; } relocs[256];
    int reloc_cnt;
} Compiler;

void emit_byte(Compiler *c, u8 b) {
    if (c->pass == 2 && c->size < c->capacity) c->code[c->size] = b;
    c->size++;
}
void emit_word(Compiler *c, u16 v) { emit_byte(c, v & 0xFF); emit_byte(c, (v>>8)&0xFF); }
void emit_dword(Compiler *c, u32 v) { emit_word(c, v & 0xFFFF); emit_word(c, (v>>16)&0xFFFF); }

void add_reloc(Compiler *c, int size, const char *label, int relative) {
    if (c->reloc_cnt < 256) {
        c->relocs[c->reloc_cnt].offset = c->size;
        c->relocs[c->reloc_cnt].size = size;
        my_strcpy(c->relocs[c->reloc_cnt].label, label);
        c->relocs[c->reloc_cnt].relative = relative;
        c->reloc_cnt++;
    }
}

/* forward declarations */
int compile_line(Compiler *c, const char *line);

/* SIB encoding support */
static void emit_sib(Compiler *c, int is_store, const char *reg_name, const char *sib_expr) {
    /* sib_expr: @base[,index[,scale[,disp]]] */
    int ri = reg_find(reg_name);
    if (ri < 0) return;
    u8 reg_idx = regs[ri].idx;
    int reg_size = regs[ri].size;

    /* parse sib_expr (skip '@') */
    const char *p = sib_expr + 1; /* skip '@' */
    char base_name[32] = {0}, index_name[32] = {0}, disp_str[64] = {0};
    int scale = 0, has_disp = 0, is_label = 0;
    s32 disp_val = 0;

    /* extract comma-separated fields */
    char field[4][64];
    int nf = 0;
    while (*p) {
        while (my_isspace(*p)) p++;
        if (*p == 0) break;
        int k = 0;
        while (*p && *p != ',' && k < 63) field[nf][k++] = *p++;
        field[nf][k] = 0;
        nf++;
        if (*p == ',') p++;
    }
    /* field0 = base, field1 = index, field2 = scale, field3 = disp */
    if (nf < 1) return; /* need at least base */
    my_strcpy(base_name, field[0]);
    if (nf >= 2) my_strcpy(index_name, field[1]);
    if (nf >= 3) scale = my_atoi(field[2]);
    if (nf >= 4) {
        my_strcpy(disp_str, field[3]);
        has_disp = 1;
        if (disp_str[0] >= '0' && disp_str[0] <= '9' || disp_str[0] == '-') {
            disp_val = my_atoi(disp_str);
            is_label = 0;
        } else {
            is_label = 1;
        }
    }

    int base_idx = reg_find(base_name);
    if (base_idx < 0) return;
    u8 base = regs[base_idx].idx;

    int index_idx = -1;
    u8 index = 4; /* no index */
    if (index_name[0] && !(index_name[0]=='0' && index_name[1]==0)) { /* not "0" */
        index_idx = reg_find(index_name);
        if (index_idx >= 0) index = regs[index_idx].idx;
    }

    /* scale encoding: 0->1,1->2,2->4,3->8; reverse from user scale */
    u8 scale_enc = 0;
    if (scale == 1) scale_enc = 0;
    else if (scale == 2) scale_enc = 1;
    else if (scale == 4) scale_enc = 2;
    else if (scale == 8) scale_enc = 3;

    /* address size prefix if 16-bit mode (SIB normally 32-bit addr, so add 67h if bits=16) */
    if (c->bits_mode == 16) emit_byte(c, 0x67);

    /* operand size prefix if needed */
    if ((reg_size == 2 && c->bits_mode == 32) || (reg_size == 4 && c->bits_mode == 16))
        emit_byte(c, 0x66);

    u8 opcode = (reg_size == 1) ? (is_store ? 0x88 : 0x8A) : (is_store ? 0x89 : 0x8B);
    emit_byte(c, opcode);

    /* ModRM: mod bits depend on displacement */
    u8 mod = 0;
    int use_disp8 = 0, use_disp32 = 0;
    if (!has_disp && !is_label) {
        if (base == 5 && index == 4) { /* [ebp] needs disp8=0 */
            mod = 1; disp_val = 0; use_disp8 = 1;
        } else {
            mod = 0;
        }
    } else if (is_label) {
        mod = (base == 5 && index == 4) ? 0 : 2;
        use_disp32 = 1;
    } else { /* explicit displacement */
        if (disp_val >= -128 && disp_val <= 127) {
            mod = 1; use_disp8 = 1;
        } else {
            mod = 2; use_disp32 = 1;
        }
    }

    emit_byte(c, (mod << 6) | (reg_idx << 3) | 4); /* r/m=4 for SIB */
    emit_byte(c, (scale_enc << 6) | ((index & 7) << 3) | (base & 7));

    if (use_disp8) emit_byte(c, (u8)(disp_val & 0xFF));
    if (use_disp32) {
        if (is_label) {
            add_reloc(c, 4, disp_str, 0);
            emit_dword(c, 0); /* placeholder */
        } else {
            emit_dword(c, (u32)disp_val);
        }
    }
}

/* ---- instruction length estimation and emission ---- */
static int parse_and_emit_mov(Compiler *c, const char *rest) {
    /* rest is "dst, src" */
    Scanner sc;
    scan_init(&sc, rest);
    if (!scan_next(&sc)) return -1;
    char *dst = sc.token;
    if (!scan_next(&sc)) return -1; /* skip comma */
    if (!scan_next(&sc)) return -1;
    char *src = sc.token;

    if (my_strncmp(dst, "DX,[", 4) == 0) {
        char var[64]; int j = 4, k = 0;
        while (dst[j] && dst[j]!=']' && k<63) var[k++]=dst[j++];
        var[k]=0;
        if (c->bits_mode==32) { emit_byte(c,0x8B); emit_byte(c,0x15); add_reloc(c,4,var,0); emit_dword(c,0); }
        else { emit_byte(c,0x8B); emit_byte(c,0x16); add_reloc(c,2,var,0); emit_word(c,0); }
        return 0;
    }

    /* reg, reg or reg, imm */
    int dst_idx = reg_find(dst);
    if (dst_idx < 0) return -1;
    u8 dst_id = regs[dst_idx].idx;
    int dst_sz = regs[dst_idx].size;

    int src_idx = reg_find(src);
    if (src_idx >= 0) { /* reg to reg */
        u8 src_id = regs[src_idx].idx;
        int src_sz = regs[src_idx].size;
        if (dst_sz != src_sz) return -1;
        if ((dst_sz == 2 && c->bits_mode == 32) || (dst_sz == 4 && c->bits_mode == 16))
            emit_byte(c, 0x66);
        emit_byte(c, dst_sz == 1 ? 0x88 : 0x89);
        emit_byte(c, (3 << 6) | (src_id << 3) | dst_id);
        return 0;
    } else { /* immediate */
        s32 imm = my_atoi(src);
        if (dst_sz == 1) {
            emit_byte(c, 0xB0 + dst_id);
            emit_byte(c, (u8)imm);
        } else {
            if ((dst_sz == 2 && c->bits_mode == 32) || (dst_sz == 4 && c->bits_mode == 16))
                emit_byte(c, 0x66);
            emit_byte(c, 0xB8 + dst_id);
            if (dst_sz == 2) emit_word(c, (u16)imm);
            else emit_dword(c, (u32)imm);
        }
        return 0;
    }
}

/* first pass: estimate size and record labels */
int pass_size(Compiler *c, const char *source) {
    const char *p = source;
    char line[256];
    while (*p) {
        int i = 0;
        while (*p && *p != '\n' && i < 255) line[i++] = *p++;
        line[i] = 0;
        if (*p == '\n') p++;
        /* strip comment */
        char *sc = line;
        while (*sc && *sc != ';') sc++;
        *sc = 0;
        /* trim */
        while (i > 0 && my_isspace(line[i-1])) line[--i] = 0;
        if (i == 0) continue;
        if (my_strncmp(line, "bits ", 5) == 0) {
            if (my_strcmp(line+5, "16") == 0) c->bits_mode = 16;
            else if (my_strcmp(line+5, "32") == 0) c->bits_mode = 32;
            continue;
        }
        if (my_strncmp(line, "equ ", 4) == 0) {
            char name[64]; s32 val;
            Scanner scn; scan_init(&scn, line+4);
            if (scan_next(&scn)) { my_strcpy(name, scn.token); }
            if (scan_next(&scn)) { val = my_atoi(scn.token); }
            equ_add(name, val);
            continue;
        }
        if (line[0] == '@') {
            char name[64]; int j = 1;
            while (line[j] && line[j] != ':') { name[j-1] = line[j]; j++; }
            name[j-1] = 0;
            label_add(name, c->size);
            continue;
        }
        if (my_strncmp(line, "var ", 4) == 0 || my_strncmp(line, "str ", 4) == 0 ||
            my_strncmp(line, "db ", 3) == 0 || my_strncmp(line, "dw ", 3) == 0 ||
            my_strncmp(line, "dd ", 3) == 0 || my_strncmp(line, "times ", 6) == 0)
            continue; /* data directives processed later */
        Compiler dummy = *c;
        dummy.pass = 1;
        dummy.code = NULL; dummy.capacity = 0; dummy.size = 0;
        if (compile_line(&dummy, line) < 0) return -1;
        c->size += dummy.size;
    }
    return c->size;
}

int compile_line(Compiler *c, const char *line) {
    Scanner sc; scan_init(&sc, line);
    if (!scan_next(&sc)) return 0;
    char *instr = sc.token;
    int pidx = 2; /* dx/edx index */
    u8 p = (u8)pidx;

    if (my_strcmp(instr, "bits") == 0) {
        if (scan_next(&sc)) {
            if (my_strcmp(sc.token, "16") == 0) c->bits_mode = 16;
            else if (my_strcmp(sc.token, "32") == 0) c->bits_mode = 32;
        }
        return 0;
    }
    if (instr[0] == '@') return 0;

    if (instr[0] == '#') {
        char *val = instr+1; if (*val==0 && scan_next(&sc)) val = sc.token;
        s32 imm = my_atoi(val);
        if (c->bits_mode == 16) { emit_byte(c,0xBA); emit_word(c,imm); }
        else { emit_byte(c,0xBA); emit_dword(c,imm); }
        return 0;
    }

    /* $, ~ with possible SIB form */
    if (instr[0] == '$' || instr[0] == '~') {
        char reg_name[32];
        if (instr[1] == 0) {
            if (!scan_next(&sc)) return -1;
            my_strcpy(reg_name, sc.token);
        } else {
            my_strcpy(reg_name, instr+1);
        }
        /* check for comma => SIB */
        const char *rem = sc.p;
        while (my_isspace(*rem)) rem++;
        if (*rem == ',') {
            scan_next(&sc); /* consume comma */
            /* read rest of line as sib expression */
            char sib_expr[128] = {0};
            int idx = 0;
            const char *q = sc.p;
            while (*q && *q != ';' && idx < 127) sib_expr[idx++] = *q++;
            sib_expr[idx] = 0;
            emit_sib(c, instr[0]=='$', reg_name, sib_expr);
            return 0;
        } else {
            int ri = reg_find(reg_name);
            if (ri < 0) return -1;
            u8 idx = regs[ri].idx;
            int sz = regs[ri].size;
            if (instr[0] == '$') {
                if (c->bits_mode == 16) emit_byte(c,0x67);
                emit_byte(c,0x88); emit_byte(c, (0<<6) | (idx<<3) | p);
            } else {
                if (c->bits_mode == 16) emit_byte(c,0x67);
                if (sz == 1) emit_byte(c,0x8A);
                else {
                    if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
                    emit_byte(c,0x8B);
                }
                emit_byte(c, (0<<6) | (idx<<3) | p);
            }
            return 0;
        }
    }

    if (instr[0] == '%') {
        char *reg = instr+1; if (*reg==0 && scan_next(&sc)) reg = sc.token;
        if (is_seg_reg(reg)) {
            int si = reg_find(reg) - 24;
            emit_byte(c,0x8C);
            emit_byte(c, (3<<6) | (si<<3) | p);
        } else {
            int ri = reg_find(reg);
            if (ri < 0) return -1;
            u8 idx = regs[ri].idx;
            int sz = regs[ri].size;
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c,0x89);
            emit_byte(c, (3<<6) | (idx<<3) | p);
        }
        return 0;
    }
    if (instr[0] == '=') {
        char *reg = instr+1; if (*reg==0 && scan_next(&sc)) reg = sc.token;
        if (is_seg_reg(reg)) {
            int si = reg_find(reg) - 24;
            emit_byte(c,0x8E);
            emit_byte(c, (3<<6) | (si<<3) | p);
        } else {
            int ri = reg_find(reg);
            if (ri < 0) return -1;
            u8 idx = regs[ri].idx;
            int sz = regs[ri].size;
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c,0x89);
            emit_byte(c, (3<<6) | (p<<3) | idx);
        }
        return 0;
    }
    if (instr[0] == '&') {
        char *var = instr+1; if (*var==0 && scan_next(&sc)) var = sc.token;
        if (c->bits_mode == 32) {
            emit_byte(c,0x8D); emit_byte(c,0x15);
            add_reloc(c, 4, var, 0); emit_dword(c,0);
        } else {
            emit_byte(c,0x8D); emit_byte(c,0x16);
            add_reloc(c, 2, var, 0); emit_word(c,0);
        }
        return 0;
    }
    if (instr[0] == '^') {
        char *reg = instr+1; if (*reg==0 && scan_next(&sc)) reg = sc.token;
        int ri = reg_find(reg);
        if (ri < 0) return -1;
        int sz = regs[ri].size;
        if (sz == 1) emit_byte(c,0xEC);
        else {
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c,0xED);
        }
        return 0;
    }
    if (instr[0] == '*') {
        char *reg = instr+1; if (*reg==0 && scan_next(&sc)) reg = sc.token;
        int ri = reg_find(reg);
        if (ri < 0) return -1;
        int sz = regs[ri].size;
        if (sz == 1) emit_byte(c,0xEE);
        else {
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c,0xEF);
        }
        return 0;
    }
    /* jump */
    if (instr[0] == '!') {
        char *lbl = instr+1; if (*lbl==0 && scan_next(&sc)) lbl = sc.token;
        emit_byte(c,0xE9);
        add_reloc(c, c->bits_mode==32?4:2, lbl, 1);
        emit_dword(c,0);
        return 0;
    }
    if (instr[0] == '?') {
        char cond = 0;
        char *lbl = NULL;
        int is_neg = 0;
        if (my_strlen(instr) >= 3 && my_toupper(instr[1]) == 'N') {
            is_neg = 1;
            cond = my_toupper(instr[2]);
            lbl = instr+3;
        } else if (my_strlen(instr) >= 2) {
            cond = my_toupper(instr[1]);
            lbl = instr+2;
        } else {
            /* ? alone: cmp byte [dx],0; jz */
            if (c->bits_mode==16) emit_byte(c,0x67);
            emit_byte(c,0x80); emit_byte(c,0x3A); emit_byte(c,0x00);
            if (!scan_next(&sc)) return -1;
            emit_byte(c,0x0F); emit_byte(c,0x84);
            add_reloc(c, c->bits_mode==32?4:2, sc.token, 1);
            emit_dword(c,0);
            return 0;
        }
        if (*lbl==0 && scan_next(&sc)) lbl = sc.token;
        u8 op2;
        if (is_neg) {
            static const u8 ncc[256] = {['Z']=0x85, ['C']=0x83, ['O']=0x81, ['S']=0x89, ['P']=0x8B};
            op2 = ncc[(int)cond];
        } else {
            static const u8 jcc[256] = {['Z']=0x84, ['C']=0x82, ['O']=0x80, ['S']=0x88, ['P']=0x8A, ['G']=0x8F, ['L']=0x8C};
            op2 = jcc[(int)cond];
        }
        if (!op2) return -1;
        emit_byte(c,0x0F); emit_byte(c,op2);
        add_reloc(c, c->bits_mode==32?4:2, lbl, 1);
        emit_dword(c,0);
        return 0;
    }

    if (my_strcmp(instr, "call") == 0) {
        if (scan_next(&sc)) {
            char *t = sc.token;
            if (t[0] == '*') {
                int ri = reg_find(t+1);
                if (ri < 0) return -1;
                emit_byte(c,0xFF); emit_byte(c, (3<<6) | (2<<3) | regs[ri].idx);
            } else {
                emit_byte(c,0xE8);
                add_reloc(c, c->bits_mode==32?4:2, t, 1);
                emit_dword(c,0);
            }
        }
        return 0;
    }

    if (my_strcmp(instr, ">") == 0) { emit_byte(c,0x40+p); return 0; }
    if (my_strcmp(instr, "<") == 0) { emit_byte(c,0x48+p); return 0; }
    if (my_strcmp(instr, "+") == 0) { if(c->bits_mode==16) emit_byte(c,0x67); emit_byte(c,0xFE); emit_byte(c,0x02); return 0; }
    if (my_strcmp(instr, "-") == 0) { if(c->bits_mode==16) emit_byte(c,0x67); emit_byte(c,0xFE); emit_byte(c,0x0A); return 0; }
    if (my_strcmp(instr, "cli")==0) { emit_byte(c,0xFA); return 0; }
    if (my_strcmp(instr, "sti")==0) { emit_byte(c,0xFB); return 0; }
    if (my_strcmp(instr, "hlt")==0) { emit_byte(c,0xF4); return 0; }
    if (my_strcmp(instr, "iret")==0) { emit_byte(c,0xCF); return 0; }
    if (my_strcmp(instr, "pushad")==0) { emit_byte(c,0x60); return 0; }
    if (my_strcmp(instr, "popad")==0) { emit_byte(c,0x61); return 0; }
    if (my_strcmp(instr, "pushf")==0) { emit_byte(c,0x9C); return 0; }
    if (my_strcmp(instr, "popf")==0) { emit_byte(c,0x9D); return 0; }
    if (my_strcmp(instr, "pusha")==0) { emit_byte(c,0x60); return 0; }
    if (my_strcmp(instr, "popa")==0) { emit_byte(c,0x61); return 0; }
    if (my_strcmp(instr, "ret")==0) { emit_byte(c,0xC3); return 0; }
    if (my_strcmp(instr, "nop")==0) { emit_byte(c,0x90); return 0; }

    if (my_strcmp(instr, "push")==0 || my_strcmp(instr, "pop")==0) {
        int ispush = (instr[0]=='p' && instr[1]=='u');
        char *op = NULL;
        if (scan_next(&sc)) op = sc.token;
        if (!op) {
            if (ispush) emit_byte(c,0x50+p); else emit_byte(c,0x58+p);
        } else if (is_seg_reg(op)) {
            int si = reg_find(op) - 24;
            if (my_strcmp(op, "fs")==0 || my_strcmp(op, "FS")==0) { emit_byte(c,0x0F); emit_byte(c,ispush?0xA0:0xA1); }
            else if (my_strcmp(op, "gs")==0 || my_strcmp(op, "GS")==0) { emit_byte(c,0x0F); emit_byte(c,ispush?0xA8:0xA9); }
            else {
                static const u8 penc[] = {0x06,0x0E,0x16,0x1E};
                static const u8 popc[] = {0x07,0x0F,0x17,0x1F};
                emit_byte(c, ispush ? penc[si] : popc[si]);
            }
        } else {
            int ri = reg_find(op);
            if (ri < 0) return -1;
            int sz = regs[ri].size;
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c, ispush ? 0x50+regs[ri].idx : 0x58+regs[ri].idx);
        }
        return 0;
    }

    if (my_strcmp(instr, "movb")==0) {
        if (!scan_next(&sc)) return -1;
        if (!scan_next(&sc)) return -1;
        s32 imm = my_atoi(sc.token);
        if (c->bits_mode==16) emit_byte(c,0x67);
        emit_byte(c,0xC6); emit_byte(c,0x02); emit_byte(c,imm);
        return 0;
    }
    if (my_strcmp(instr, "movd")==0) {
        if (!scan_next(&sc)) return -1;
        if (!scan_next(&sc)) return -1;
        s32 imm = my_atoi(sc.token);
        if (c->bits_mode==16) emit_byte(c,0x67);
        emit_byte(c,0xC7); emit_byte(c,0x02);
        if (c->bits_mode==32) emit_dword(c,imm); else emit_word(c,imm);
        return 0;
    }
    if (my_strcmp(instr, "add")==0 || my_strcmp(instr, "sub")==0 || my_strcmp(instr, "cmp")==0) {
        if (!scan_next(&sc)) return -1;
        if (!scan_next(&sc)) return -1;
        s32 imm = my_atoi(sc.token);
        u8 mod = (instr[0]=='a')?0 : ((instr[0]=='s')?5:7);
        if (c->bits_mode==16) {
            emit_byte(c,0x81); emit_byte(c, (3<<6)|(mod<<3)|p); emit_word(c,imm);
        } else {
            emit_byte(c,0x81); emit_byte(c, (3<<6)|(mod<<3)|p); emit_dword(c,imm);
        }
        return 0;
    }

    if (my_strcmp(instr, "inc")==0 || my_strcmp(instr, "dec")==0) {
        if (!scan_next(&sc)) return -1;
        char *reg = sc.token;
        int isinc = (instr[0]=='i');
        if (my_strcmp(reg, "DX")==0 || my_strcmp(reg, "dx")==0) {
            emit_byte(c, (isinc?0x40:0x48)+p);
        } else {
            int ri = reg_find(reg);
            if (ri < 0) return -1;
            int sz = regs[ri].size;
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c, (isinc?0x40:0x48)+regs[ri].idx);
        }
        return 0;
    }

    if (my_strcmp(instr, "int")==0) {
        if (!scan_next(&sc)) return -1;
        emit_byte(c,0xCD); emit_byte(c, my_atoi(sc.token));
        return 0;
    }
    if (my_strcmp(instr, "lidt")==0) {
        if (!scan_next(&sc)) return -1;
        if (c->bits_mode==32) { emit_byte(c,0x0F); emit_byte(c,0x01); emit_byte(c,0x1D); add_reloc(c,4,sc.token,0); emit_dword(c,0); }
        else { emit_byte(c,0x0F); emit_byte(c,0x01); emit_byte(c,0x1E); add_reloc(c,2,sc.token,0); emit_word(c,0); }
        return 0;
    }
    if (my_strcmp(instr, "out")==0) {
        if (!scan_next(&sc)) return -1; char *port = sc.token;
        if (!scan_next(&sc)) return -1;
        if (!scan_next(&sc)) return -1; char *reg = sc.token;
        int ri = reg_find(reg);
        if (ri < 0) return -1;
        int sz = regs[ri].size;
        s32 portnum = my_atoi(port);
        if (sz == 1) { emit_byte(c,0xE6); emit_byte(c,portnum); }
        else {
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c,0xE7); emit_byte(c,portnum);
        }
        return 0;
    }
    if (my_strcmp(instr, "in")==0) {
        if (!scan_next(&sc)) return -1; char *reg = sc.token;
        if (!scan_next(&sc)) return -1;
        if (!scan_next(&sc)) return -1; char *port = sc.token;
        int ri = reg_find(reg);
        if (ri < 0) return -1;
        int sz = regs[ri].size;
        s32 portnum = my_atoi(port);
        if (sz == 1) { emit_byte(c,0xE4); emit_byte(c,portnum); }
        else {
            if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
            emit_byte(c,0xE5); emit_byte(c,portnum);
        }
        return 0;
    }

    if (my_strcmp(instr, "mov")==0) {
        /* remainder of line */
        const char *rest = sc.p;
        char tmp[128]; int i=0;
        while (*rest && *rest != ';' && i<127) tmp[i++] = *rest++;
        tmp[i]=0;
        return parse_and_emit_mov(c, tmp);
    }

    /* mul / imul / div / idiv */
    if (my_strcmp(instr, "mul")==0 || my_strcmp(instr, "imul")==0 ||
        my_strcmp(instr, "div")==0 || my_strcmp(instr, "idiv")==0) {
        if (!scan_next(&sc)) return -1;
        char *op = sc.token;
        int ri = reg_find(op);
        if (ri < 0) return -1; /* memory not supported */
        int sz = regs[ri].size;
        u8 reg_field;
        if (instr[0]=='m') reg_field = 4;
        else if (instr[0]=='i' && instr[1]=='m') reg_field = 5;
        else if (instr[0]=='d' && instr[1]=='i') reg_field = 6;
        else reg_field = 7; /* idiv */
        if ((sz==2 && c->bits_mode==32) || (sz==4 && c->bits_mode==16)) emit_byte(c,0x66);
        emit_byte(c, sz==1 ? 0xF6 : 0xF7);
        emit_byte(c, (3<<6) | (reg_field<<3) | regs[ri].idx);
        return 0;
    }

    return -1; /* unknown instruction */
}

void gen_data(Compiler *c, const char *line) {
    Scanner sc; scan_init(&sc, line);
    if (!scan_next(&sc)) return;
    char *instr = sc.token;

    if (my_strcmp(instr, "var")==0) {
        if (!scan_next(&sc)) return; char *name = sc.token;
        if (!scan_next(&sc)) return; int sz = my_atoi(sc.token);
        label_add(name, c->size);
        while (sz-- > 0) emit_byte(c,0);
        return;
    }
    if (my_strcmp(instr, "str")==0) {
        if (!scan_next(&sc)) return; char *name = sc.token;
        const char *q = sc.p;
        while (*q && *q!='"') q++;
        if (!*q) return;
        q++;
        label_add(name, c->size);
        while (*q && *q!='"') { emit_byte(c, *q); q++; }
        emit_byte(c,0);
        return;
    }
    if (my_strcmp(instr, "db")==0) {
        while (scan_next(&sc)) {
            char *tok = sc.token;
            if (tok[0]=='"') {
                for (int i=1; tok[i] && tok[i]!='"'; i++) emit_byte(c,tok[i]);
            } else {
                emit_byte(c, my_atoi(tok));
            }
        }
        return;
    }
    if (my_strcmp(instr, "dw")==0) {
        while (scan_next(&sc)) emit_word(c, my_atoi(sc.token));
        return;
    }
    if (my_strcmp(instr, "dd")==0) {
        while (scan_next(&sc)) emit_dword(c, my_atoi(sc.token));
        return;
    }
    if (my_strcmp(instr, "times")==0) {
        if (!scan_next(&sc)) return;
        char *cnt_expr = sc.token;
        if (!scan_next(&sc)) return;
        char *cmd = sc.token;
        if (!scan_next(&sc)) return;
        char *val = sc.token;
        s32 cnt = expr_eval(cnt_expr, c->size + c->org_base, c->org_base);
        if (cnt < 0) return;
        if (my_strcmp(cmd, "db")==0) {
            u8 byte = my_atoi(val);
            while (cnt-- > 0) emit_byte(c, byte);
        } else if (my_strcmp(cmd, "dw")==0) {
            u16 word = my_atoi(val);
            while (cnt-- > 0) emit_word(c, word);
        } else if (my_strcmp(cmd, "dd")==0) {
            u32 dword = my_atoi(val);
            while (cnt-- > 0) emit_dword(c, dword);
        }
        return;
    }
}

int microasm_compile(const char *source, u8 *out_buf, int max_size, u32 org_base) {
    Compiler comp;
    comp.code = out_buf;
    comp.capacity = max_size;
    comp.size = 0;
    comp.bits_mode = 32;
    comp.org_base = org_base;
    comp.pass = 1;
    comp.reloc_cnt = 0;
    label_cnt = 0;
    equ_cnt = 0;

    comp.pass = 1;
    if (pass_size(&comp, source) < 0) return -1;

    comp.pass = 2;
    comp.size = 0;
    const char *p = source;
    char line[256];
    while (*p) {
        int i = 0;
        while (*p && *p != '\n' && i < 255) line[i++] = *p++;
        line[i] = 0;
        if (*p == '\n') p++;
        char *sc = line; while (*sc && *sc != ';') sc++; *sc = 0;
        while (i > 0 && my_isspace(line[i-1])) line[--i] = 0;
        if (i == 0) continue;
        if (my_strncmp(line, "bits ",5)==0) { continue; }
        if (my_strncmp(line, "equ ",4)==0) { continue; }
        if (line[0] == '@') { continue; }
        if (my_strncmp(line,"var ",4)==0 || my_strncmp(line,"str ",4)==0 ||
            my_strncmp(line,"db ",3)==0 || my_strncmp(line,"dw ",3)==0 ||
            my_strncmp(line,"dd ",3)==0 || my_strncmp(line,"times ",6)==0) {
            continue;
        }
        if (compile_line(&comp, line) < 0) return -1;
    }
    /* emit data */
    p = source;
    while (*p) {
        int i = 0;
        while (*p && *p != '\n' && i < 255) line[i++] = *p++;
        line[i] = 0; if (*p == '\n') p++;
        char *sc = line; while (*sc && *sc != ';') sc++; *sc = 0;
        while (i > 0 && my_isspace(line[i-1])) line[--i] = 0;
        if (i == 0) continue;
        if (my_strncmp(line,"var ",4)==0 || my_strncmp(line,"str ",4)==0 ||
            my_strncmp(line,"db ",3)==0 || my_strncmp(line,"dw ",3)==0 ||
            my_strncmp(line,"dd ",3)==0 || my_strncmp(line,"times ",6)==0) {
            gen_data(&comp, line);
        }
    }
    /* resolve relocs */
    for (int i = 0; i < comp.reloc_cnt; i++) {
        u32 target = 0;
        u32 lbl_addr = label_lookup(comp.relocs[i].label);
        if (lbl_addr == 0xFFFFFFFF) {
            s32 ev;
            if (equ_lookup(comp.relocs[i].label, &ev)) lbl_addr = (u32)ev;
            else return -1;
        }
        target = lbl_addr + org_base;
        u32 offset = comp.relocs[i].offset;
        if (comp.relocs[i].relative) {
            s32 rel = (s32)(target - (offset + comp.relocs[i].size));
            for (int j = 0; j < comp.relocs[i].size; j++) {
                out_buf[offset + j] = (rel >> (j*8)) & 0xFF;
            }
        } else {
            for (int j = 0; j < comp.relocs[i].size; j++) {
                out_buf[offset + j] = (target >> (j*8)) & 0xFF;
            }
        }
    }
    return comp.size;
}

#ifdef TEST
#include <stdio.h>
int main() {
    const char *src = 
        "bits 16\n"
        "@start:\n"
        "#0x8000\n"
        "=sp\n"
        "%cs\n"
        "=ds\n"
        "jc read_err\n"
        "@read_err\n"
        "hlt\n"
        "!read_err\n"
        "times 510-($-$$) db 0\n"
        "dw 0xAA55\n";
    u8 buf[1024];
    int size = microasm_compile(src, buf, 1024, 0x7C00);
    if (size < 0) { printf("error\n"); return 1; }
    for (int i=0; i<size; i++) {
        printf("%02x ", buf[i]);
        if ((i+1)%16==0) printf("\n");
    }
    return 0;
}
#endif