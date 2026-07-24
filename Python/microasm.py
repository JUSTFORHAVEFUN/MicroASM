#!/usr/bin/env python3
"""
microasm2.py - µASM v2 Compiler (NASM + Binary/COM)
Usage:
  python microasm2.py input.masm                      -> NASM to stdout
  python microasm2.py -o output.asm input.masm        -> NASM to file
  python microasm2.py -bin -org 0x7C00 input.masm     -> binary to stdout
  python microasm2.py -bin -o boot.bin input.masm     -> binary file
  python microasm2.py -com input.masm                 -> .com binary to stdout
  python microasm2.py -com -o program.com input.masm  -> .com file
"""

import sys
import os
import struct
from typing import List, Dict, Optional, Tuple

# ---------- register information ----------
class RegInfo:
    def __init__(self, name: str, size: int, idx: int):
        self.name = name      # lowercase NASM name
        self.size = size      # 1, 2, 4 bytes
        self.idx = idx        # register encoding 0-7

REGISTERS = {
    "AL":  RegInfo("al",  1, 0), "CL":  RegInfo("cl",  1, 1), "DL":  RegInfo("dl",  1, 2), "BL":  RegInfo("bl",  1, 3),
    "AH":  RegInfo("ah",  1, 4), "CH":  RegInfo("ch",  1, 5), "DH":  RegInfo("dh",  1, 6), "BH":  RegInfo("bh",  1, 7),
    "AX":  RegInfo("ax",  2, 0), "CX":  RegInfo("cx",  2, 1), "DX":  RegInfo("dx",  2, 2), "BX":  RegInfo("bx",  2, 3),
    "SP":  RegInfo("sp",  2, 4), "BP":  RegInfo("bp",  2, 5), "SI":  RegInfo("si",  2, 6), "DI":  RegInfo("di",  2, 7),
    "EAX": RegInfo("eax", 4, 0), "ECX": RegInfo("ecx", 4, 1), "EDX": RegInfo("edx", 4, 2), "EBX": RegInfo("ebx", 4, 3),
    "ESP": RegInfo("esp", 4, 4), "EBP": RegInfo("ebp", 4, 5), "ESI": RegInfo("esi", 4, 6), "EDI": RegInfo("edi", 4, 7),
    # segment registers (for recognition, encoding handled separately)
    "CS":  RegInfo("cs",  2, 1), "DS":  RegInfo("ds",  2, 3), "ES":  RegInfo("es",  2, 0), "SS":  RegInfo("ss",  2, 2),
    "FS":  RegInfo("fs",  2, 4), "GS":  RegInfo("gs",  2, 5)
}

SEG_IDX = {"es":0, "cs":1, "ss":2, "ds":3, "fs":4, "gs":5}

def is_reg(s: str) -> bool:
    return s.upper() in REGISTERS

def get_reg(s: str) -> RegInfo:
    return REGISTERS[s.upper()]

def is_seg(s: str) -> bool:
    return s.lower() in SEG_IDX

def seg_idx(s: str) -> int:
    return SEG_IDX[s.lower()]

# ---------- utility functions ----------
def trim(s: str) -> str:
    return s.strip()

def parse_int(s: str) -> int:
    if not s:
        return 0
    s = s.strip()
    # handle suffixes K, M, G
    mult = 1
    if s[-1].upper() == 'K':
        mult = 1024
        s = s[:-1]
    elif s[-1].upper() == 'M':
        mult = 1024 * 1024
        s = s[:-1]
    elif s[-1].upper() == 'G':
        mult = 1024 * 1024 * 1024
        s = s[:-1]
    # parse as integer (supports 0x hex, etc.)
    try:
        base = 10
        if s.startswith('0x') or s.startswith('0X'):
            base = 16
        elif s.startswith('0') and len(s) > 1:
            base = 8
        return int(s, base) * mult
    except ValueError:
        return 0

# ---------- expression evaluator (supports $ and $$) ----------
class ExprEval:
    def __init__(self, s: str, cur_addr: int, base_addr: int):
        self.s = s
        self.pos = 0
        self.cur_addr = cur_addr
        self.base_addr = base_addr

    def eval(self) -> int:
        val = self.parse_add()
        if self.pos != len(self.s):
            raise RuntimeError("extra characters in expression")
        return val

    def peek(self) -> str:
        return self.s[self.pos] if self.pos < len(self.s) else ''

    def get(self) -> str:
        c = self.s[self.pos]
        self.pos += 1
        return c

    def skip_spaces(self):
        while self.peek() == ' ':
            self.get()

    def parse_add(self) -> int:
        v = self.parse_mul()
        while True:
            self.skip_spaces()
            if self.peek() == '+':
                self.get()
                v += self.parse_mul()
            elif self.peek() == '-':
                self.get()
                v -= self.parse_mul()
            else:
                break
        return v

    def parse_mul(self) -> int:
        v = self.parse_primary()
        while True:
            self.skip_spaces()
            if self.peek() == '*':
                self.get()
                v *= self.parse_primary()
            elif self.peek() == '/':
                self.get()
                d = self.parse_primary()
                if d == 0:
                    raise RuntimeError("div by zero")
                v //= d
            else:
                break
        return v

    def parse_primary(self) -> int:
        self.skip_spaces()
        if self.peek() == '(':
            self.get()
            v = self.parse_add()
            self.skip_spaces()
            if self.get() != ')':
                raise RuntimeError("missing ')'")
            return v
        if self.peek() == '$':
            self.get()
            if self.peek() == '$':
                self.get()
                return self.base_addr
            return self.cur_addr
        # number
        num = ''
        if self.peek() in ('-', '+'):
            num += self.get()
        while self.peek().isdigit() or self.peek() in ('x', 'X', 'a', 'b', 'c', 'd', 'e', 'f', 'A', 'B', 'C', 'D', 'E', 'F'):
            num += self.get()
        if not num:
            raise RuntimeError("expected number")
        return parse_int(num)

# ---------- SIB helper ----------
class SIBInfo:
    def __init__(self):
        self.base = 0
        self.index = 4   # 4 means no index
        self.scale = 0
        self.disp = 0
        self.disp_present = False
        self.is_label = False
        self.label = ""

def parse_sib(expr: str) -> SIBInfo:
    # expr starts with '@'
    info = SIBInfo()
    s = expr[1:]  # remove '@'
    parts = [trim(p) for p in s.split(',')]
    if not parts or len(parts) > 4:
        raise RuntimeError("invalid SIB expression: " + expr)

    # field 1: base (required)
    if not is_reg(parts[0]):
        raise RuntimeError("SIB base must be register: " + parts[0])
    info.base = get_reg(parts[0]).idx

    # field 2: index (optional)
    if len(parts) >= 2:
        idx = parts[1]
        if idx in ('0', '1'):
            info.index = 4   # no index
        elif is_reg(idx):
            info.index = get_reg(idx).idx
        else:
            raise RuntimeError("SIB index must be register or 0/1: " + idx)

    # field 3: scale / extra register
    if len(parts) >= 3:
        scl = parts[2]
        if is_reg(scl):
            # "変位 +" mode: Index = field3, Base = field1, Scale = 0
            info.index = get_reg(scl).idx
            info.scale = 0
        else:
            if scl.startswith('#'):
                scl = scl[1:]
            scale_val = parse_int(scl)
            if scale_val == 1:   info.scale = 0
            elif scale_val == 2: info.scale = 1
            elif scale_val == 4: info.scale = 2
            elif scale_val == 8: info.scale = 3
            else:
                raise RuntimeError("invalid scale: " + scl)

    # field 4: displacement
    if len(parts) >= 4:
        disp_str = parts[3]
        if disp_str in ('0', '1'):
            info.disp_present = False
            info.disp = 0
        else:
            info.disp_present = True
            if disp_str.startswith('#'):
                disp_str = disp_str[1:]
            try:
                info.disp = parse_int(disp_str)
                info.is_label = False
            except ValueError:
                info.is_label = True
                info.label = disp_str
                info.disp = 0
    return info

# ---------- main compiler class ----------
class MicroAsmCompiler:
    def __init__(self):
        self.bits_mode = 32
        self.org_base = 0
        self.lines: List[str] = []
        self.equ_map: Dict[str, int] = {}
        self.var_names: set = set()
        self.label_addr: Dict[str, int] = {}
        self.var_addr: Dict[str, int] = {}
        self.output: List[str] = []
        self.data_out: List[str] = []
        self.code_buf = bytearray()
        self.relocs: List[Tuple[int, int, str, bool]] = []  # (offset, size, label, relative)

    def ptr(self) -> str:
        return "dx" if self.bits_mode == 16 else "edx"

    def dsize(self) -> str:
        return "word" if self.bits_mode == 16 else "dword"

    def pidx(self) -> int:
        return 2  # DX/EDX index

    def resolve_equ(self, s: str) -> str:
        if s in self.equ_map:
            return str(self.equ_map[s])
        return s

    def is_data_directive(self, line: str) -> bool:
        return (line.startswith("var ") or line.startswith("str ") or
                line.startswith("db ")  or line.startswith("dw ") or
                line.startswith("dd ")  or line.startswith("times "))

    def read_lines(self, in_stream):
        self.lines.clear()
        self.equ_map.clear()
        self.var_names.clear()
        self.label_addr.clear()
        self.var_addr.clear()
        for raw in in_stream:
            line = raw.split(';')[0].strip()
            if not line:
                continue
            self.lines.append(line)
            if line.startswith("equ "):
                parts = line[4:].split()
                if len(parts) >= 2:
                    self.equ_map[parts[0]] = parse_int(parts[1])
            elif line.startswith("var "):
                parts = line[4:].split()
                if parts:
                    self.var_names.add(parts[0])
            elif line.startswith("str "):
                parts = line[4:].split()
                if parts:
                    self.var_names.add(parts[0])

    def first_pass_binary(self):
        off = 0
        for raw in self.lines:
            line = raw.split(';')[0].strip()
            if not line:
                continue
            if line.startswith("equ "):
                continue
            if line == "bits 16":
                self.bits_mode = 16
                continue
            if line == "bits 32":
                self.bits_mode = 32
                continue
            if line.startswith('@'):
                lbl = line[1:]
                if lbl.endswith(':'):
                    lbl = lbl[:-1]
                self.label_addr[lbl] = off
                continue
            if self.is_data_directive(line):
                continue
            off += self.instr_length(line)

    def instr_length(self, line: str) -> int:
        parts = line.split()
        instr = parts[0]
        if instr == "bits":
            return 0
        if instr in (">", "<"):
            return 1
        if instr in ("+", "-"):
            return 3 if self.bits_mode == 16 else 2
        if instr in ("cli","sti","hlt","iret","pushad","popad","pushf","popf","pusha","popa","ret","nop"):
            return 1
        if instr in ("push","pop"):
            # approximate: push/pop with operand can have prefixes
            if len(parts) > 1:
                op = parts[1]
                if is_seg(op):
                    return 2 if op.lower() in ("fs","gs") else 1
                else:
                    # reg or mem (not fully handled for mem but safe)
                    return 1
            else:
                return 1  # push/pop dx
        if instr[0] == '#':
            return 1 + (4 if self.bits_mode == 32 else 2)
        if instr[0] in ('$', '~'):
            rest = line[len(instr):].strip()
            if ',' not in rest:
                return 2 + (1 if self.bits_mode == 16 else 0)
            else:
                # SIB form, rough overestimate
                return 2 + 1 + 4 + (1 if self.bits_mode == 16 else 0)
        if instr[0] in ('=', '%'):
            r = parts[1] if len(parts) > 1 else ""
            if is_seg(r):
                return 2
            ri = get_reg(r)
            base = 2
            if ri.size == 2 and self.bits_mode == 32:
                base = 3
            if ri.size == 4 and self.bits_mode == 16:
                base = 3
            return base
        if instr[0] == '&':
            return 2 + (4 if self.bits_mode == 32 else 2)
        if instr[0] in ('^', '*'):
            r = parts[1] if len(parts) > 1 else ""
            ri = get_reg(r)
            base = 1
            if ri.size > 1:
                if (self.bits_mode == 32 and ri.size == 2) or (self.bits_mode == 16 and ri.size == 4):
                    base = 2
                else:
                    base = 1
            return base
        if instr[0] == '!':
            return 1 + (4 if self.bits_mode == 32 else 2)
        if instr[0] == '?':
            if len(instr) >= 3 and instr[1].upper() == 'N':
                return 2 + (4 if self.bits_mode == 32 else 2)
            if len(instr) >= 2:
                return 2 + (4 if self.bits_mode == 32 else 2)
            return 4 + (4 if self.bits_mode == 32 else 2)
        if instr == "call":
            if len(parts) > 1 and parts[1].startswith('*'):
                return 2
            return 1 + (4 if self.bits_mode == 32 else 2)
        if instr in ("add","sub","cmp"):
            return 2 + (4 if self.bits_mode == 32 else 2)
        if instr == "movb":
            return (3 if self.bits_mode == 16 else 2) + 1
        if instr == "movd":
            return (3 if self.bits_mode == 16 else 2) + (4 if self.bits_mode == 32 else 2)
        if instr == "mov":
            rest = line[3:].strip()
            if rest.startswith("DX,["):
                return 2 + (4 if self.bits_mode == 32 else 2)
            dst_src = [trim(x) for x in rest.split(',')]
            if len(dst_src) >= 2 and is_reg(dst_src[0]) and is_reg(dst_src[1]):
                rd = get_reg(dst_src[0])
                base = 2
                if rd.size == 2 and self.bits_mode == 32:
                    base = 3
                if rd.size == 4 and self.bits_mode == 16:
                    base = 3
                return base
            elif len(dst_src) >= 1 and is_reg(dst_src[0]):
                rd = get_reg(dst_src[0])
                if rd.size == 1:
                    return 2
                base = 1
                if self.bits_mode == 32 and rd.size == 2:
                    base = 3
                if self.bits_mode == 16 and rd.size == 4:
                    base = 3
                return base + (2 if rd.size == 2 else 4)
        if instr in ("inc","dec"):
            if len(parts) > 1 and parts[1] == "DX":
                return 1
            if len(parts) > 1:
                ri = get_reg(parts[1])
                if (ri.size == 2 and self.bits_mode == 32) or (ri.size == 4 and self.bits_mode == 16):
                    return 2
                return 1
        if instr == "int":
            return 2
        if instr == "lidt":
            return 3 + (4 if self.bits_mode == 32 else 2)
        if instr == "out":
            return 2 + (1 if self.bits_mode == 32 else 0)  # rough
        if instr == "in":
            return 2 + (1 if self.bits_mode == 32 else 0)
        # mul/imul/div/idiv
        if instr in ("mul","imul","div","idiv"):
            # opcode + modrm, plus possible prefix
            if len(parts) > 1 and is_reg(parts[1]):
                ri = get_reg(parts[1])
                if (ri.size == 2 and self.bits_mode == 32) or (ri.size == 4 and self.bits_mode == 16):
                    return 3  # prefix + opcode + modrm
                return 2
            return 2  # assume mem same length? approximation
        raise RuntimeError("unknown length for: " + line)

    # ---------- NASM generation ----------
    def nasm_line(self, line: str):
        parts = line.split()
        instr = parts[0]
        if instr == "bits":
            mode = parts[1] if len(parts) > 1 else ""
            if mode == "16":
                self.bits_mode = 16
                self.output.append("[bits 16]")
            elif mode == "32":
                self.bits_mode = 32
                self.output.append("[bits 32]")
            return
        if instr.startswith('@'):
            lbl = instr[1:]
            if lbl.endswith(':'):
                lbl = lbl[:-1]
            self.output.append(lbl + ":")
            return
        # handle single-char instructions
        if instr[0] == '#':
            self.output.append("    mov " + self.ptr() + ", " + self.resolve_equ(parts[1] if len(parts)>1 else ""))
        elif instr[0] == '$':
            rest = line[1:].strip()
            if ',' in rest:
                # SIB store: $ reg, @...
                p = [trim(x) for x in rest.split(',', 1)]
                if len(p) >= 2 and p[1].startswith('@'):
                    self.nasm_sib_load_store(True, p[1], get_reg(p[0]))
                else:
                    raise RuntimeError("invalid $ SIB form")
            else:
                ri = get_reg(rest)
                sz = "byte" if ri.size==1 else ("word" if ri.size==2 else self.dsize())
                self.output.append(f"    mov {sz} [{self.ptr()}], {ri.name}")
        elif instr[0] == '~':
            rest = line[1:].strip()
            if ',' in rest:
                p = [trim(x) for x in rest.split(',', 1)]
                if len(p) >= 2 and p[1].startswith('@'):
                    self.nasm_sib_load_store(False, p[1], get_reg(p[0]))
                else:
                    raise RuntimeError("invalid ~ SIB form")
            else:
                ri = get_reg(rest)
                sz = "byte" if ri.size==1 else ("word" if ri.size==2 else self.dsize())
                self.output.append(f"    mov {ri.name}, {sz} [{self.ptr()}]")
        elif instr[0] == '=':
            r = parts[1] if len(parts)>1 else ""
            if is_seg(r):
                self.output.append(f"    mov {r.lower()}, {self.ptr()}")
            else:
                self.output.append(f"    mov {get_reg(r).name}, {self.ptr()}")
        elif instr[0] == '%':
            r = parts[1] if len(parts)>1 else ""
            if is_seg(r):
                self.output.append(f"    mov {self.ptr()}, {r.lower()}")
            else:
                self.output.append(f"    mov {self.ptr()}, {get_reg(r).name}")
        elif instr[0] == '&':
            self.output.append(f"    lea {self.ptr()}, [{parts[1] if len(parts)>1 else ''}]")
        elif instr[0] == '^':
            r = parts[1] if len(parts)>1 else ""
            self.output.append(f"    in {get_reg(r).name}, {self.ptr()}")
        elif instr[0] == '*':
            r = parts[1] if len(parts)>1 else ""
            self.output.append(f"    out {self.ptr()}, {get_reg(r).name}")
        elif instr[0] == '!':
            lbl = parts[1] if len(parts)>1 else ""
            self.output.append(f"    jmp {lbl}")
        elif instr[0] == '?':
            if len(instr) >= 3 and instr[1].upper() == 'N':
                c = instr[2].upper()
                lbl = instr[3:] if len(instr)>3 else (parts[1] if len(parts)>1 else "")
                ncc = {"Z":"jnz","C":"jnc","O":"jno","S":"jns","P":"jnp"}
                if c in ncc:
                    self.output.append(f"    {ncc[c]} {lbl}")
            elif len(instr) >= 2:
                c = instr[1].upper()
                lbl = instr[2:] if len(instr)>2 else (parts[1] if len(parts)>1 else "")
                jcc = {"Z":"jz","C":"jc","O":"jo","S":"js","P":"jp","G":"jg","L":"jl"}
                if c in jcc:
                    self.output.append(f"    {jcc[c]} {lbl}")
            else:
                lbl = parts[1] if len(parts)>1 else ""
                self.output.append(f"    cmp byte [{self.ptr()}], 0")
                self.output.append(f"    jz {lbl}")
        elif instr == "call":
            t = parts[1] if len(parts)>1 else ""
            if t.startswith('*'):
                self.output.append(f"    call {get_reg(t[1:]).name}")
            else:
                self.output.append(f"    call {self.resolve_equ(t)}")
        elif instr == ">":
            self.output.append(f"    inc {self.ptr()}")
        elif instr == "<":
            self.output.append(f"    dec {self.ptr()}")
        elif instr == "+":
            self.output.append(f"    inc byte [{self.ptr()}]")
        elif instr == "-":
            self.output.append(f"    dec byte [{self.ptr()}]")
        elif instr in ("cli","sti","hlt","iret","pushad","popad","pushf","popf","pusha","popa","ret","nop"):
            self.output.append(f"    {instr}")
        elif instr in ("push","pop"):
            op = parts[1] if len(parts)>1 else ""
            if instr == "push":
                if not op:
                    self.output.append(f"    push {self.ptr()}")
                elif is_seg(op):
                    self.output.append(f"    push {op.lower()}")
                else:
                    self.output.append(f"    push {get_reg(op).name}")
            else:
                if not op:
                    self.output.append(f"    pop {self.ptr()}")
                elif is_seg(op):
                    self.output.append(f"    pop {op.lower()}")
                else:
                    self.output.append(f"    pop {get_reg(op).name}")
        elif instr in ("add","sub","cmp"):
            rest = line[len(instr):].strip()
            if rest.startswith("DX,"):
                imm = rest[3:].strip()
                self.output.append(f"    {instr} {self.ptr()}, {self.resolve_equ(imm)}")
            else:
                raise RuntimeError("unsupported " + instr)
        elif instr == "movb":
            rest = line[4:].strip()
            if rest.startswith("[DX],"):
                imm = rest[5:].strip()
                self.output.append(f"    mov byte [{self.ptr()}], {self.resolve_equ(imm)}")
            else:
                raise RuntimeError("invalid movb")
        elif instr == "movd":
            rest = line[4:].strip()
            if rest.startswith("[DX],"):
                imm = rest[5:].strip()
                self.output.append(f"    mov {self.dsize()} [{self.ptr()}], {self.resolve_equ(imm)}")
            else:
                raise RuntimeError("invalid movd")
        elif instr == "mov":
            rest = line[3:].strip()
            if rest.startswith("DX,["):
                var = rest[4:].strip()
                if var.endswith(']'):
                    var = var[:-1]
                self.output.append(f"    mov {self.ptr()}, [{var}]")
            else:
                dst_src = [trim(x) for x in rest.split(',', 1)]
                if len(dst_src) >= 2 and is_reg(dst_src[0]) and is_reg(dst_src[1]):
                    rd, rs = get_reg(dst_src[0]), get_reg(dst_src[1])
                    if rd.size != rs.size:
                        raise RuntimeError("size mismatch")
                    self.output.append(f"    mov {rd.name}, {rs.name}")
                elif len(dst_src) >= 1 and is_reg(dst_src[0]):
                    self.output.append(f"    mov {get_reg(dst_src[0]).name}, {self.resolve_equ(dst_src[1])}")
                else:
                    raise RuntimeError("invalid mov")
        elif instr in ("inc","dec"):
            reg = parts[1] if len(parts)>1 else ""
            if reg == "DX":
                self.output.append(f"    {instr} {self.ptr()}")
            else:
                self.output.append(f"    {instr} {get_reg(reg).name}")
        elif instr == "int":
            num = parts[1] if len(parts)>1 else ""
            self.output.append(f"    int {self.resolve_equ(num)}")
        elif instr == "lidt":
            addr = parts[1] if len(parts)>1 else ""
            self.output.append(f"    lidt [{addr}]")
        elif instr == "out":
            rest = line[3:].strip()
            imm, reg = [trim(x) for x in rest.split(',')]
            self.output.append(f"    out {self.resolve_equ(imm)}, {get_reg(reg).name}")
        elif instr == "in":
            rest = line[2:].strip()
            reg, imm = [trim(x) for x in rest.split(',')]
            self.output.append(f"    in {get_reg(reg).name}, {self.resolve_equ(imm)}")
        elif instr in ("mul","imul","div","idiv"):
            op = parts[1] if len(parts)>1 else ""
            if not op:
                raise RuntimeError("missing operand for " + instr)
            if is_reg(op):
                self.output.append(f"    {instr} {get_reg(op).name}")
            else:
                # could be memory, but for now just label
                self.output.append(f"    {instr} byte [{op}]")  # default to byte? actually size needed, we'll trust user
        elif instr == "var":
            rest = line[3:].strip()
            name, sz_str = rest.split()
            sz = int(sz_str)
            self.data_out.append(f"{name}: times {sz} db 0")
        elif instr == "str":
            rest = line[3:].strip()
            # find name and quoted string
            if '"' in rest:
                name, text = rest.split(None, 1)
                self.data_out.append(f"{name}: db {text}, 0")
        elif instr in ("times","db","dw","dd"):
            self.output.append(f"    {line}")
        else:
            raise RuntimeError("unknown instruction: " + instr)

    def nasm_sib_load_store(self, is_store: bool, sib_expr: str, ri: RegInfo):
        sib = parse_sib(sib_expr)
        sz = "byte" if ri.size==1 else ("word" if ri.size==2 else self.dsize())
        # build address string
        base_name = "eax"
        for k, v in REGISTERS.items():
            if v.size == 4 and v.idx == sib.base:
                base_name = v.name
                break
        addr = "[" + base_name
        if sib.index != 4:
            idx_name = "eax"
            for k, v in REGISTERS.items():
                if v.size == 4 and v.idx == sib.index:
                    idx_name = v.name
                    break
            if sib.scale > 0:
                addr += f"+{idx_name}*{1<<sib.scale}"
            else:
                addr += f"+{idx_name}"
        if sib.is_label:
            addr += "+" + sib.label
        elif sib.disp_present:
            if sib.disp >= 0:
                addr += "+" + str(sib.disp)
            else:
                addr += str(sib.disp)
        addr += "]"
        if is_store:
            self.output.append(f"    mov {sz} {addr}, {ri.name}")
        else:
            self.output.append(f"    mov {ri.name}, {sz} {addr}")

    def compile_nasm(self, in_stream) -> str:
        self.read_lines(in_stream)
        self.output.clear()
        self.data_out.clear()
        self.output.append("section .text")
        self.output.append("global _start")
        self.output.append("_start:")
        for raw in self.lines:
            line = raw.split(';')[0].strip()
            if not line:
                continue
            try:
                self.nasm_line(line)
            except Exception as e:
                print(f"NASM error: {e} in: {raw}", file=sys.stderr)
                sys.exit(1)
        if self.data_out:
            self.output.append("")
            self.output.append("section .data")
            self.output.extend(self.data_out)
        return "\n".join(self.output) + "\n"

    # ---------- binary encoding ----------
    def encode_sib_load_store(self, is_store: bool, sib_expr: str, ri: RegInfo):
        sib = parse_sib(sib_expr)
        # helper lambdas
        def b(val): self.code_buf.append(val)
        def w32(val):
            self.code_buf.extend(struct.pack('<I', val & 0xFFFFFFFF))
        def abs_reloc(sz, lbl):
            off = len(self.code_buf)
            self.code_buf.extend(b'\x00' * sz)
            self.relocs.append((off, sz, lbl, False))

        # address size prefix
        if self.bits_mode == 16:
            b(0x67)
        # operand size prefix
        need_opsize = (ri.size == 2 and self.bits_mode == 32) or (ri.size == 4 and self.bits_mode == 16)
        if need_opsize:
            b(0x66)

        opcode = 0x88 if ri.size == 1 else (0x89 if is_store else 0x8B)
        if not is_store:
            opcode = 0x8A if ri.size == 1 else 0x8B
        b(opcode)

        # ModRM
        mod = 0
        use_disp8 = False
        use_disp32 = False
        disp_val = sib.disp
        if not sib.disp_present and not sib.is_label:
            if sib.base == 5 and sib.index == 4:
                mod = 1
                disp_val = 0
                use_disp8 = True
            else:
                mod = 0
        elif sib.is_label:
            mod = 0 if (sib.base == 5 and sib.index == 4) else 2
            use_disp32 = True
        else:
            if -128 <= sib.disp <= 127:
                mod = 1
                use_disp8 = True
            else:
                mod = 2
                use_disp32 = True

        b((mod << 6) | (ri.idx << 3) | 4)  # r/m=4 for SIB
        b((sib.scale << 6) | ((sib.index & 7) << 3) | (sib.base & 7))

        if use_disp8:
            b(disp_val & 0xFF)
        elif use_disp32:
            if sib.is_label:
                abs_reloc(4, sib.label)
            else:
                w32(disp_val)

    def encode_line(self, line: str):
        parts = line.split()
        instr = parts[0]

        if instr == "bits":
            mode = parts[1] if len(parts)>1 else ""
            if mode == "16": self.bits_mode = 16
            elif mode == "32": self.bits_mode = 32
            return

        def b(val): self.code_buf.append(val)
        def w16(val): self.code_buf.extend(struct.pack('<H', val & 0xFFFF))
        def w32(val): self.code_buf.extend(struct.pack('<I', val & 0xFFFFFFFF))
        def abs_reloc(sz, lbl):
            off = len(self.code_buf)
            self.code_buf.extend(b'\x00' * sz)
            self.relocs.append((off, sz, lbl, False))
        def rel_reloc(sz, lbl):
            off = len(self.code_buf)
            self.code_buf.extend(b'\x00' * sz)
            self.relocs.append((off, sz, lbl, True))

        p = self.pidx()

        if instr[0] == '#':
            val = parts[1] if len(parts)>1 else ""
            imm = parse_int(self.resolve_equ(val))
            if self.bits_mode == 16:
                b(0xBA); w16(imm)
            else:
                b(0xBA); w32(imm)
        elif instr[0] == '$':
            rest = line[1:].strip()
            if ',' in rest:
                # SIB store
                p1, p2 = [trim(x) for x in rest.split(',', 1)]
                if p2.startswith('@'):
                    self.encode_sib_load_store(True, p2, get_reg(p1))
                else:
                    raise RuntimeError("expected @SIB expression")
            else:
                ri = get_reg(rest)
                if self.bits_mode == 16: b(0x67)
                b(0x88); b((0<<6) | (ri.idx<<3) | p)
        elif instr[0] == '~':
            rest = line[1:].strip()
            if ',' in rest:
                p1, p2 = [trim(x) for x in rest.split(',', 1)]
                if p2.startswith('@'):
                    self.encode_sib_load_store(False, p2, get_reg(p1))
                else:
                    raise RuntimeError("expected @SIB expression")
            else:
                ri = get_reg(rest)
                if self.bits_mode == 16: b(0x67)
                if ri.size == 1:
                    b(0x8A)
                else:
                    if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                        b(0x66)
                    b(0x8B)
                b((0<<6) | (ri.idx<<3) | p)
        elif instr[0] == '%':
            r = parts[1] if len(parts)>1 else ""
            if is_seg(r):
                s = seg_idx(r)
                b(0x8C); b((3<<6) | (s<<3) | p)
            else:
                ri = get_reg(r)
                if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                    b(0x66)
                b(0x89); b((3<<6) | (ri.idx<<3) | p)
        elif instr[0] == '=':
            r = parts[1] if len(parts)>1 else ""
            if is_seg(r):
                s = seg_idx(r)
                b(0x8E); b((3<<6) | (s<<3) | p)
            else:
                ri = get_reg(r)
                if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                    b(0x66)
                b(0x89); b((3<<6) | (p<<3) | ri.idx)
        elif instr[0] == '&':
            var = parts[1] if len(parts)>1 else ""
            if self.bits_mode == 32:
                b(0x8D); b(0x15); abs_reloc(4, var)
            else:
                b(0x8D); b(0x16); abs_reloc(2, var)
        elif instr[0] == '^':
            r = parts[1] if len(parts)>1 else ""
            ri = get_reg(r)
            if ri.size == 1:
                b(0xEC)
            else:
                if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                    b(0x66)
                b(0xED)
        elif instr[0] == '*':
            r = parts[1] if len(parts)>1 else ""
            ri = get_reg(r)
            if ri.size == 1:
                b(0xEE)
            else:
                if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                    b(0x66)
                b(0xEF)
        elif instr[0] == '!':
            lbl = parts[1] if len(parts)>1 else ""
            b(0xE9)
            rel_reloc(4 if self.bits_mode==32 else 2, lbl)
        elif instr[0] == '?':
            if len(instr) >= 3 and instr[1].upper() == 'N':
                c = instr[2].upper()
                lbl = instr[3:] if len(instr)>3 else (parts[1] if len(parts)>1 else "")
                ncc_enc = {'Z':0x85, 'C':0x83, 'O':0x81, 'S':0x89, 'P':0x8B}
                if c in ncc_enc:
                    b(0x0F); b(ncc_enc[c])
                    rel_reloc(4 if self.bits_mode==32 else 2, lbl)
            elif len(instr) >= 2:
                c = instr[1].upper()
                lbl = instr[2:] if len(instr)>2 else (parts[1] if len(parts)>1 else "")
                jcc_enc = {'Z':0x84, 'C':0x82, 'O':0x80, 'S':0x88, 'P':0x8A, 'G':0x8F, 'L':0x8C}
                if c in jcc_enc:
                    b(0x0F); b(jcc_enc[c])
                    rel_reloc(4 if self.bits_mode==32 else 2, lbl)
            else:
                if self.bits_mode == 16: b(0x67)
                b(0x80); b(0x3A); b(0x00)
                lbl = parts[1] if len(parts)>1 else ""
                b(0x0F); b(0x84)
                rel_reloc(4 if self.bits_mode==32 else 2, lbl)
        elif instr == "call":
            t = parts[1] if len(parts)>1 else ""
            if t.startswith('*'):
                ri = get_reg(t[1:])
                b(0xFF); b((3<<6) | (2<<3) | ri.idx)
            else:
                b(0xE8)
                rel_reloc(4 if self.bits_mode==32 else 2, t)
        elif instr in (">", "<"):
            b((0x40 if instr==">" else 0x48) + p)
        elif instr in ("+", "-"):
            if self.bits_mode == 16: b(0x67)
            b(0xFE); b(0x02 if instr=="+" else 0x0A)
        elif instr in ("cli","sti","hlt","iret","pushad","popad","pushf","popf","pusha","popa","ret","nop"):
            opcodes = {"cli":0xFA,"sti":0xFB,"hlt":0xF4,"iret":0xCF,"pushad":0x60,"popad":0x61,
                       "pushf":0x9C,"popf":0x9D,"pusha":0x60,"popa":0x61,"ret":0xC3,"nop":0x90}
            b(opcodes[instr])
        elif instr in ("push","pop"):
            op = parts[1] if len(parts)>1 else ""
            if instr == "push":
                if not op:
                    b(0x50 + p)
                elif is_seg(op):
                    s = seg_idx(op)
                    if op.lower() == "fs":
                        b(0x0F); b(0xA0)
                    elif op.lower() == "gs":
                        b(0x0F); b(0xA8)
                    else:
                        enc = [0x06,0x0E,0x16,0x1E]
                        b(enc[s])
                else:
                    ri = get_reg(op)
                    if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                        b(0x66)
                    b(0x50 + ri.idx)
            else:  # pop
                if not op:
                    b(0x58 + p)
                elif is_seg(op):
                    s = seg_idx(op)
                    if op.lower() == "fs":
                        b(0x0F); b(0xA1)
                    elif op.lower() == "gs":
                        b(0x0F); b(0xA9)
                    else:
                        enc = [0x07,0x0F,0x17,0x1F]
                        b(enc[s])
                else:
                    ri = get_reg(op)
                    if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                        b(0x66)
                    b(0x58 + ri.idx)
        elif instr in ("add","sub","cmp"):
            rest = line[len(instr):].strip()
            if rest.startswith("DX,"):
                imm = parse_int(self.resolve_equ(rest[3:].strip()))
                mod_field = {"add":0, "sub":5, "cmp":7}[instr]
                if self.bits_mode == 16:
                    b(0x81); b((3<<6) | (mod_field<<3) | p); w16(imm)
                else:
                    b(0x81); b((3<<6) | (mod_field<<3) | p); w32(imm)
        elif instr == "movb":
            rest = line[4:].strip()
            if rest.startswith("[DX],"):
                imm = parse_int(self.resolve_equ(rest[5:].strip()))
                if self.bits_mode == 16: b(0x67)
                b(0xC6); b(0x02); b(imm)
        elif instr == "movd":
            rest = line[4:].strip()
            if rest.startswith("[DX],"):
                imm = parse_int(self.resolve_equ(rest[5:].strip()))
                if self.bits_mode == 16: b(0x67)
                b(0xC7); b(0x02)
                if self.bits_mode == 32:
                    w32(imm)
                else:
                    w16(imm)
        elif instr == "mov":
            rest = line[3:].strip()
            if rest.startswith("DX,["):
                var = rest[4:].strip()
                if var.endswith(']'):
                    var = var[:-1]
                if self.bits_mode == 32:
                    b(0x8B); b(0x15); abs_reloc(4, var)
                else:
                    b(0x8B); b(0x16); abs_reloc(2, var)
            else:
                dst_src = [trim(x) for x in rest.split(',', 1)]
                if len(dst_src) >= 2 and is_reg(dst_src[0]) and is_reg(dst_src[1]):
                    rd, rs = get_reg(dst_src[0]), get_reg(dst_src[1])
                    if rd.size != rs.size:
                        raise RuntimeError("size mismatch")
                    if (rd.size==2 and self.bits_mode==32) or (rd.size==4 and self.bits_mode==16):
                        b(0x66)
                    b(0x88 if rd.size==1 else 0x89)
                    b((3<<6) | (rs.idx<<3) | rd.idx)
                elif len(dst_src) >= 1 and is_reg(dst_src[0]):
                    rd = get_reg(dst_src[0])
                    imm = parse_int(self.resolve_equ(dst_src[1]))
                    if rd.size == 1:
                        b(0xB0 + rd.idx); b(imm)
                    else:
                        if (rd.size==2 and self.bits_mode==32) or (rd.size==4 and self.bits_mode==16):
                            b(0x66)
                        b(0xB8 + rd.idx)
                        if rd.size == 2:
                            w16(imm)
                        else:
                            w32(imm)
        elif instr in ("inc","dec"):
            reg = parts[1] if len(parts)>1 else ""
            if reg == "DX":
                b((0x40 if instr=="inc" else 0x48) + p)
            else:
                ri = get_reg(reg)
                if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                    b(0x66)
                b((0x40 if instr=="inc" else 0x48) + ri.idx)
        elif instr == "int":
            num = parts[1] if len(parts)>1 else ""
            b(0xCD); b(parse_int(self.resolve_equ(num)))
        elif instr == "lidt":
            addr = parts[1] if len(parts)>1 else ""
            if self.bits_mode == 32:
                b(0x0F); b(0x01); b(0x1D); abs_reloc(4, addr)
            else:
                b(0x0F); b(0x01); b(0x1E); abs_reloc(2, addr)
        elif instr == "out":
            rest = line[3:].strip()
            imm, reg = [trim(x) for x in rest.split(',')]
            ri = get_reg(reg)
            port = parse_int(self.resolve_equ(imm))
            if ri.size == 1:
                b(0xE6); b(port)
            else:
                if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                    b(0x66)
                b(0xE7); b(port)
        elif instr == "in":
            rest = line[2:].strip()
            reg, imm = [trim(x) for x in rest.split(',')]
            ri = get_reg(reg)
            port = parse_int(self.resolve_equ(imm))
            if ri.size == 1:
                b(0xE4); b(port)
            else:
                if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                    b(0x66)
                b(0xE5); b(port)
        elif instr in ("mul","imul","div","idiv"):
            op = parts[1] if len(parts)>1 else ""
            if not op:
                raise RuntimeError("missing operand")
            if is_reg(op):
                ri = get_reg(op)
            else:
                # assume memory with size byte for now (default)
                # we'll handle memory later; for now force reg
                raise RuntimeError("memory operand not yet supported for mul/div")
            # determine reg field in ModRM
            reg_field = {"mul":4, "imul":5, "div":6, "idiv":7}[instr]
            # add operand size prefix if needed
            if (ri.size==2 and self.bits_mode==32) or (ri.size==4 and self.bits_mode==16):
                b(0x66)
            if ri.size == 1:
                b(0xF6)
            else:
                b(0xF7)
            # ModRM: mod=3 (register), reg=reg_field, r/m=ri.idx
            b((3<<6) | (reg_field<<3) | ri.idx)
        else:
            raise RuntimeError("unsupported instruction: " + instr)

    def gen_data(self, line: str):
        parts = line.split()
        instr = parts[0]
        if instr == "var":
            rest = line[3:].strip()
            name, sz_str = rest.split()
            sz = int(sz_str)
            self.var_addr[name] = len(self.code_buf)
            self.code_buf.extend(b'\x00' * sz)
        elif instr == "str":
            rest = line[3:].strip()
            name, rest2 = rest.split(None, 1)
            # extract string between quotes
            start = rest2.find('"')
            end = rest2.rfind('"')
            if start != -1 and end != -1:
                content = rest2[start+1:end]
                # handle escapes simply
                content = content.replace('\\n', '\n').replace('\\t', '\t').replace('\\\\', '\\')
                self.var_addr[name] = len(self.code_buf)
                self.code_buf.extend(content.encode('latin-1'))
                self.code_buf.append(0)
        elif instr in ("db","dw","dd"):
            rest = line[len(instr):].strip()
            items = [trim(x) for x in rest.split(',')]
            for item in items:
                if item.startswith('"'):
                    self.code_buf.extend(item.strip('"').encode('latin-1'))
                else:
                    v = parse_int(item)
                    if instr == "db":
                        self.code_buf.append(v & 0xFF)
                    elif instr == "dw":
                        self.code_buf.extend(struct.pack('<H', v & 0xFFFF))
                    elif instr == "dd":
                        self.code_buf.extend(struct.pack('<I', v & 0xFFFFFFFF))
        elif instr == "times":
            rest = line[5:].strip()
            count_expr, rest2 = rest.split(None, 1)
            cmd, val = rest2.split(None, 1)
            cur = len(self.code_buf) + self.org_base
            cnt = ExprEval(count_expr, cur, self.org_base).eval()
            if cnt < 0:
                raise RuntimeError("times count negative")
            val = val.strip()
            if cmd == "db":
                byte = parse_int(val) & 0xFF
                self.code_buf.extend(bytes([byte]) * cnt)
            elif cmd == "dw":
                word = parse_int(val) & 0xFFFF
                self.code_buf.extend(struct.pack('<H', word) * cnt)
            elif cmd == "dd":
                dword = parse_int(val) & 0xFFFFFFFF
                self.code_buf.extend(struct.pack('<I', dword) * cnt)
            else:
                raise RuntimeError("unknown times data type: " + cmd)

    def compile_bin(self, in_stream) -> bytes:
        self.read_lines(in_stream)
        self.first_pass_binary()
        self.code_buf.clear()
        self.relocs.clear()
        for raw in self.lines:
            line = raw.split(';')[0].strip()
            if not line:
                continue
            if line.startswith('@') or line.startswith("equ "):
                continue
            if self.is_data_directive(line):
                continue
            try:
                self.encode_line(line)
            except Exception as e:
                print(f"Binary error: {e} in: {raw}", file=sys.stderr)
                sys.exit(1)
        # second pass: data directives
        for raw in self.lines:
            line = raw.split(';')[0].strip()
            if not line:
                continue
            if self.is_data_directive(line):
                self.gen_data(line)
        # resolve relocations
        for off, sz, lbl, relative in self.relocs:
            target = 0
            if lbl in self.label_addr:
                target = self.label_addr[lbl] + self.org_base
            elif lbl in self.var_addr:
                target = self.var_addr[lbl] + self.org_base
            else:
                raise RuntimeError("undefined label: " + lbl)
            if relative:
                delta = target - (off + sz)
                for i in range(sz):
                    self.code_buf[off + i] = (delta >> (i*8)) & 0xFF
            else:
                for i in range(sz):
                    self.code_buf[off + i] = (target >> (i*8)) & 0xFF
        return bytes(self.code_buf)

# ---------- main ----------
def main():
    import sys
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} [-bin] [-com] [-org addr] [-o output] file.masm")
        sys.exit(1)

    input_file = None
    output_file = None
    bin_mode = False
    com_mode = False
    org = 0
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a == "-bin":
            bin_mode = True
        elif a == "-com":
            com_mode = True
            bin_mode = True  # .com implies binary output
        elif a == "-org" and i+1 < len(args):
            i += 1
            org = parse_int(args[i])
        elif a == "-o" and i+1 < len(args):
            i += 1
            output_file = args[i]
        elif a.startswith('-'):
            print(f"Unknown option: {a}", file=sys.stderr)
            sys.exit(1)
        else:
            if input_file is None:
                input_file = a
            else:
                print("Multiple input files.", file=sys.stderr)
                sys.exit(1)
        i += 1

    if input_file is None:
        print("No input file specified.", file=sys.stderr)
        sys.exit(1)

    if com_mode:
        if org == 0:
            org = 0x100

    try:
        with open(input_file, 'r') as f:
            compiler = MicroAsmCompiler()
            compiler.org_base = org
            if bin_mode:
                binary = compiler.compile_bin(f)
                if output_file:
                    with open(output_file, 'wb') as out:
                        out.write(binary)
                else:
                    sys.stdout.buffer.write(binary)
            else:
                asm_code = compiler.compile_nasm(f)
                if output_file:
                    with open(output_file, 'w') as out:
                        out.write(asm_code)
                else:
                    sys.stdout.write(asm_code)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
