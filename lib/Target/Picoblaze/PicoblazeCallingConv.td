//==- PicoblazeCallingConv.td - Calling Conventions for Picoblaze -*- tablegen -*-==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for Picoblaze architecture.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Picoblaze Return Value Calling Convention
//===----------------------------------------------------------------------===//
def RetCC_Picoblaze : CallingConv<[
  // i8 are returned in registers R15B, R14B, R13B, R12B
  CCIfType<[i8], CCAssignToReg<[Sd, Sc, Sb, Sa]>>
]>;

//===----------------------------------------------------------------------===//
// Picoblaze Argument Calling Conventions
//===----------------------------------------------------------------------===//
def CC_Picoblaze : CallingConv<[
  // Promote i8 arguments to i16.
  // CCIfType<[i8], CCPromoteToType<i16>>,

  // The first 4 integer arguments of non-varargs functions are passed in
  // integer registers.
  CCIfNotVarArg<CCIfType<[i8], CCAssignToReg<[Sd,Sc,Sb,Sa]>>>,

  // Integer values get stored in stack slots that are 2 bytes in
  // size and 2-byte aligned.
  CCIfType<[i16], CCAssignToStack<2, 2>>
]>;
