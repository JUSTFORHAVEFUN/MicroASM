OVERVIEW
--------
µASM is a minimal x86 assembler designed for bare-metal and systems programming.
Three language implementations share identical assembly syntax but offer
different output targets and use cases.

IMPLEMENTATIONS
---------------
Version   File              Language         Targets                      Best For
-------------------------------------------------------------------------------------
C++       microasm.cpp     C++17            NASM text, flat binary       Performance,
                                                                          C++ integration
Python    microasm.py      Python 3         NASM text, flat binary,      Rapid prototyping,
                                             .COM                         scripting
C         microasm.c        C (freestanding) Flat binary only             Kernels, bootloaders,
                                                                          embedded systems

BUILD & USAGE
-------------

C++ version:
  g++ -std=c++17 -O2 -o microasm microasm.cpp
  ./microasm input.masm                      -> NASM to stdout
  ./microasm -o output.asm input.masm        -> NASM to file
  ./microasm -bin -org 0x7C00 input.masm     -> binary to stdout
  ./microasm -bin -o boot.bin input.masm     -> binary file

Python version:
  python microasm.py input.masm              -> NASM to stdout
  python microasm.py -o output.asm input.masm -> NASM to file
  python microasm.py -bin -org 0x7C00 input.masm -> binary to stdout
  python microasm.py -bin -o boot.bin input.masm -> binary file
  python microasm.py -com input.masm         -> .COM to stdout (org 0x100)
  python microasm.py -com -o prog.com input.masm -> .COM file

C version (freestanding, call from C code):
  int microasm_compile(const char *source, u8 *out_buf, int max_size, u32 org_base);
  Returns binary size, or -1 on error.
  Compile test: gcc -DTEST -o microasm microasm.c

ASSEMBLY SYNTAX
---------------

Registers:
  8-bit:   AL CL DL BL AH CH DH BH
  16-bit:  AX CX DX BX SP BP SI DI
  32-bit:  EAX ECX EDX EBX ESP EBP ESI EDI
  Segment: CS DS ES SS FS GS

Pointer Register:
  In 16-bit mode: DX
  In 32-bit mode: EDX

Data Movement:
  #imm             mov edx, imm            Load immediate into DX/EDX
  $reg             mov [edx], reg          Store register to [DX/EDX]
  ~reg             mov reg, [edx]          Load register from [DX/EDX]
  =reg             mov reg, edx            Copy DX/EDX to register
  %reg             mov edx, reg            Copy register to DX/EDX
  =seg             mov seg, edx            Copy DX/EDX to segment register
  %seg             mov edx, seg            Copy segment register to DX/EDX
  &label           lea edx, [label]        Load effective address of label
  movb [DX], imm   mov byte [edx], imm     Store byte immediate
  movd [DX], imm   mov dword [edx], imm    Store dword/word immediate
  mov DX, [var]    mov edx, [var]          Load DX/EDX from variable address
  mov reg, reg     mov reg, reg            Register-to-register move
  mov reg, imm     mov reg, imm            Load immediate into any register

SIB Addressing (C++ and Python only):
  $ reg, @base              mov [base], reg
  $ reg, @base,index        mov [base+index], reg
  $ reg, @base,index,scale  mov [base+index*scale], reg
  $ reg, @base,0,0,disp     mov [base+disp], reg
  ~ reg, @base,index,scale,disp   mov reg, [base+index*scale+disp]
  Base and index must be 32-bit registers (EAX, EBX, ECX, EDX, ESI, EDI, EBP, ESP).
  Scale: 1, 2, 4, 8.
  Displacement can be an integer or a label.

Arithmetic:
  >                inc edx                 Increment DX/EDX
  <                dec edx                 Decrement DX/EDX
  +                inc byte [edx]          Increment byte at [DX/EDX]
  -                dec byte [edx]          Decrement byte at [DX/EDX]
  add DX, imm      add edx, imm            Add immediate to DX/EDX
  sub DX, imm      sub edx, imm            Subtract immediate from DX/EDX
  cmp DX, imm      cmp edx, imm            Compare DX/EDX with immediate
  inc reg          inc reg                 Increment register
  dec reg          dec reg                 Decrement register
  mul reg          mul reg                 Unsigned multiply
  imul reg         imul reg                Signed multiply
  div reg          div reg                 Unsigned divide
  idiv reg         idiv reg                Signed divide

I/O:
  ^reg             in reg, edx             Input from port DX/EDX to register
  *reg             out edx, reg            Output from register to port DX/EDX
  out imm, reg     out imm, reg            Output to immediate port
  in reg, imm      in reg, imm             Input from immediate port

Control Flow:
  !label           jmp label               Unconditional jump
  ?label           cmp byte [edx], 0       Compare [DX/EDX] with zero, then
                   jz label                jump if zero
  ?Zlabel          jz label                Jump if zero (ZF=1)
  ?Clabel          jc label                Jump if carry (CF=1)
  ?Olabel          jo label                Jump if overflow (OF=1)
  ?Slabel          js label                Jump if sign (SF=1)
  ?Plabel          jp label                Jump if parity (PF=1)
  ?Glabel          jg label                Jump if greater (signed)
  ?Llabel          jl label                Jump if less (signed)
  ?NZlabel         jnz label               Jump if not zero
  ?NClabel         jnc label               Jump if not carry
  ?NOlabel         jno label               Jump if not overflow
  ?NSlabel         jns label               Jump if not sign
  ?NPlabel         jnp label               Jump if not parity
  call label       call label              Near call
  call *reg        call reg                Indirect call
  ret              ret                     Return

Stack:
  push             push edx                Push DX/EDX
  pop              pop edx                 Pop DX/EDX
  push reg         push reg                Push register
  pop reg          pop reg                 Pop register
  push seg         push seg                Push segment register
  pop seg          pop seg                 Pop segment register
  pushad / pusha   pushad / pusha          Push all general-purpose registers
  popad / popa     popad / popa            Pop all general-purpose registers
  pushf            pushf                   Push flags
  popf             popf                    Pop flags

System:
  cli              cli                     Clear interrupt flag
  sti              sti                     Set interrupt flag
  hlt              hlt                     Halt
  iret             iret                    Interrupt return
  int imm          int imm                 Software interrupt
  lidt label       lidt [label]            Load interrupt descriptor table
  nop              nop                     No operation

Directives:
  bits 16 / bits 32        Set code generation mode
  equ name value           Define symbolic constant
  @label:                  Define a label
  var name size            Reserve uninitialized data (bytes)
  str name "text"          Define null-terminated string
  db value[,value...]      Emit bytes
  dw value[,value...]      Emit words (16-bit)
  dd value[,value...]      Emit dwords (32-bit)
  times count db/dw/dd val Repeat data directive count times
                           (supports $ and $$ in count)

  Comments start with ; and run to end of line.

EXAMPLES
--------

Boot sector (16-bit, org 0x7C00):
  bits 16
  #0x8000
  =sp
  %cs
  =ds
  @loop:
    ~al
    $al
    >+
  ?NZloop
  hlt
  times 510-($-$$) db 0
  dw 0xAA55

.COM program (Python only):
  bits 16
  #msg
  @loop:
    ~al
    ?Zdone
    ^al
    >+
  !loop
  @done:
  ret
  str msg "Hello, World!"

Using SIB (C++/Python):
  =ebx
  $ al, @ebx,esi,2,0x100    ; mov [ebx+esi*2+0x100], al
  ~ eax, @ebp,edi,4,data     ; mov eax, [ebp+edi*4+data]

ERROR HANDLING
--------------
C++ and Python versions print error messages to stderr with line information
and exit with code 1 on failure.
C version returns -1 on error (no error messages, suitable for freestanding).

LIMITATIONS
-----------
- No macro support.
- No floating-point or SSE instructions.
- C version: limited instruction set, no SIB, no reg-reg/reg-imm mov, no mul/div.
- Labels and equates are global; no local scoping.
- Single source file; no linker support.