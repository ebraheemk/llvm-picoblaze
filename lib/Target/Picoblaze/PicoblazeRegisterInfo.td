//===-- PicoblazeRegisterInfo.td - Picoblaze Register defs -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the Picoblaze register file
//===----------------------------------------------------------------------===//

// 定义了 Picoblaze 寄存器类
// 
class PicoblazeReg<bits<5> num, string n> : Register<n> {
  field bits<5> Num = num;
  let Namespace = "Picoblaze";
}



//===----------------------------------------------------------------------===//
//  Registers
//===----------------------------------------------------------------------===//

def S0  : PicoblazeReg<0,  "S0">;
def S1  : PicoblazeReg<1,  "S1">;
def S2  : PicoblazeReg<2,  "S2">;
def S3  : PicoblazeReg<3,  "S3">;
def S4  : PicoblazeReg<4,  "S4">;
def S5  : PicoblazeReg<5,  "S5">;
def S6  : PicoblazeReg<6,  "S6">;
def S7  : PicoblazeReg<7,  "S7">;
def S8  : PicoblazeReg<8,  "S8">;
def S9  : PicoblazeReg<9,  "S9">;
def Sa : PicoblazeReg<10, "Sa">;
def Sb : PicoblazeReg<11, "Sb">;
def Sc : PicoblazeReg<12, "Sc">;
def Sd : PicoblazeReg<13, "Sd">;
def BP : PicoblazeReg<14, "BP">;
def SP     : PicoblazeReg<15, "SP">;
def STATUS  : PicoblazeReg<16, "Status">;

def PC  : PicoblazeReg<17, "PC">;


def GR8 : RegisterClass<"Picoblaze", [i8], 8,
   // Volatile registers
  (add S0,S1,S2,S3,S4,S5,S6,S7,S8,S9,Sa,Sb,Sc,Sd,
   // Frame pointer, sometimes allocable
   BP,
   // Volatile, but not allocable
   SP,
   STATUS)>;

def GR16 : RegisterClass<"Picoblaze", [i16], 16,
   // Volatile registers
  ( add 
   PC
  )>
  ;
