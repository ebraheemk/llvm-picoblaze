//===-- PicoblazeInstrInfo.td - Picoblaze Instruction defs -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the Picoblaze instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "PicoblazeInstrFormats.td"

//===----------------------------------------------------------------------===//
// Type Constraints.
//===----------------------------------------------------------------------===//
class SDTCisI8<int OpNum>  : SDTCisVT<OpNum, i8>;


//===----------------------------------------------------------------------===//
// Type Profiles.
//===----------------------------------------------------------------------===//
def SDT_PicoblazeCall         : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
def SDT_PicoblazeCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i8>]>;
def SDT_PicoblazeCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i8>, SDTCisVT<1, i8>]>;

def SDT_PicoblazeWrapper      : SDTypeProfile<1, 1, [SDTCisSameAs<0, 1>,
                                                  SDTCisPtrTy<0>]>;
def SDT_PicoblazeCmp          : SDTypeProfile<0, 2, [SDTCisSameAs<0, 1>]>;
def SDT_PicoblazeBrCC         : SDTypeProfile<0, 2, [SDTCisVT<0, OtherVT>,
                                                  SDTCisVT<1, i8>]>;
def SDT_PicoblazeSelectCC     : SDTypeProfile<1, 3, [SDTCisSameAs<0, 1>,
                                                  SDTCisSameAs<1, 2>, 
                                                  SDTCisVT<3, i8>]>;
def SDT_PicoblazeShift        : SDTypeProfile<1, 2, [SDTCisSameAs<0, 1>,
                                                  SDTCisI8<2>]>;

//===----------------------------------------------------------------------===//
// Picoblaze Specific Node Definitions.
//===----------------------------------------------------------------------===//
def Picoblazeretflag  : SDNode<"PicoblazeISD::RET_FLAG", SDTNone,
                       [SDNPHasChain, SDNPOptInGlue]>;
def Picoblazeretiflag : SDNode<"PicoblazeISD::RETI_FLAG", SDTNone,
                       [SDNPHasChain, SDNPOptInGlue]>;

def Picoblazerra     : SDNode<"PicoblazeISD::RRA", SDTIntUnaryOp, []>;
def Picoblazerla     : SDNode<"PicoblazeISD::RLA", SDTIntUnaryOp, []>;
def Picoblazerrc     : SDNode<"PicoblazeISD::RRC", SDTIntUnaryOp, []>;
def PicoblazeTest    : SDNode<"PicoblazeISD::TEST", SDTIntUnaryOp, []>;
def Picoblazecall    : SDNode<"PicoblazeISD::CALL", SDT_PicoblazeCall,
                     [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue, SDNPVariadic]>;

def Picoblazecallseq_start :
                 SDNode<"ISD::CALLSEQ_START", SDT_PicoblazeCallSeqStart,
                        [SDNPHasChain, SDNPOutGlue]>;

def Picoblazecallseq_end :
                 SDNode<"ISD::CALLSEQ_END",   SDT_PicoblazeCallSeqEnd,
                        [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def PicoblazeWrapper : SDNode<"PicoblazeISD::Wrapper", SDT_PicoblazeWrapper>;
def Picoblazecmp     : SDNode<"PicoblazeISD::CMP", SDT_PicoblazeCmp, [SDNPOutGlue]>;
def Picoblazebrcc    : SDNode<"PicoblazeISD::BR_CC", SDT_PicoblazeBrCC,
                            [SDNPHasChain, SDNPInGlue]>;
def Picoblazeselectcc: SDNode<"PicoblazeISD::SELECT_CC", SDT_PicoblazeSelectCC,
                            [SDNPInGlue]>;
def Picoblazeshl     : SDNode<"PicoblazeISD::SHL", SDT_PicoblazeShift, []>;
def Picoblazesra     : SDNode<"PicoblazeISD::SRA", SDT_PicoblazeShift, []>;
def Picoblazesrl     : SDNode<"PicoblazeISD::SRL", SDT_PicoblazeShift, []>;

//def SDTPicoblazePBP  : SDTypeProfile<1, 0,[SDTCisI8<0>]>;
//def SDTPicoblazePBP2 : SDTypeProfile<1, 2,[SDTCisI8<0>,SDTCisI8<1>,SDTCisI8<2>]>;

def SDTPicoblazeGETBP  : SDTypeProfile<1, 0,[SDTCisI8<0>]>;

//def PicoblazePBP     : SDNode<"PicoblazeISD::PBP", SDTPicoblazePBP>;
//def PicoblazePBP2    : SDNode<"PicoblazeISD::PBP2", SDTPicoblazePBP2,[]>;
def PicoblazeGETBP    : SDNode<"PicoblazeISD::GETBP", SDTPicoblazeGETBP>;

let Defs = [SP, BP], Uses = [SP] in {
def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i8imm:$amt),
                              "#ADJCALLSTACKDOWN",
                              [(Picoblazecallseq_start timm:$amt)]>;
def ADJCALLSTACKUP   : Pseudo<(outs), (ins i8imm:$amt1, i8imm:$amt2),
                              "#ADJCALLSTACKUP",
                              [(Picoblazecallseq_end timm:$amt1, timm:$amt2)]>;


}
def STORETOSTACK :   Pseudo<(outs), (ins),
                              ";#STORETOSTACK",
                              []>;
def LOADFROMSTACK :   Pseudo<(outs), (ins),
                              ";#LOADFROMSTACK",
                              []>;
def REGXXSTACKEND :   Pseudo<(outs), (ins),
                              ";#REGXXSTACKEND",
                              []>;

let Uses = [BP] in {
def PicoblazeGETBP_INSTR : Pseudo<(outs GR8:$dst), (ins ),
                              "LOAD $dst,BP ; \t#PicoblazeGETBP_INSTR",
                              [(set GR8:$dst,(PicoblazeGETBP))]>;
					}

//===----------------------------------------------------------------------===//
// Picoblaze Operand Definitions.
//===----------------------------------------------------------------------===//

// Address operands
def memsrc : Operand<i8> {
  let PrintMethod = "printSrcMemOperand";
  let MIOperandInfo = (ops GR8, i8imm);
}

def memdst : Operand<i8> {
  let PrintMethod = "printSrcMemOperand";
  let MIOperandInfo = (ops GR8, i8imm);
}

// Short jump targets have OtherVT type and are printed as pcrel imm values.
def jmptarget : Operand<OtherVT> {
  let PrintMethod = "printPCRelImmOperand";
}

// Operand for printing out a condition code.
def cc : Operand<i8> {
  let PrintMethod = "printCCOperand";
}

//===----------------------------------------------------------------------===//
// Picoblaze Complex Pattern Definitions.
//===----------------------------------------------------------------------===//

def addrframeindex :  ComplexPattern<iPTR, 1, "SelectAddrFrameindex", [frameindex], [SDNPWantParent]>;
def addr           :  ComplexPattern<iPTR, 2, "SelectAddr", [], []>;
def addrindexed    :  ComplexPattern<iPTR, 1, "SelectAddrIndexed", [], []>; 
def addrunindexed  :  ComplexPattern<iPTR, 1, "SelectAddrUnindexed", [], []>; 
def addrcall       : ComplexPattern<iPTR, 1, "SelectAddrCall", [], []>; 
//def addri        :  ComplexPattern<iPTR, 2, "SelectAddrI", [], []>;
//def addrr        :  ComplexPattern<iPTR, 2, "SelectAddrR", [], []>;
//===----------------------------------------------------------------------===//
// Pattern Fragments
//def  and_su : PatFrag<(ops node:$lhs, node:$rhs), (and node:$lhs, node:$rhs), [{
//  return N->hasOneUse();
//}]>;

//===----------------------------------------------------------------------===//
// Instruction list..






let neverHasSideEffects = 1 in
def NOP : Pseudo<(outs), (ins), "nop", []>;

//===----------------------------------------------------------------------===//
//  Control Flow Instructions...
//

// 
let isReturn = 1, isTerminator = 1, isBarrier = 1  in {
  def RET  : FBRANCH<0b101010, (outs), (ins), "RETURN",  [(Picoblazeretflag)]>;
 // def RET  : FBRANCH<0b101010, (outs), (ins), "RETURN",[]>;
  
}



// FIXME: expand opcode & cond field for branches!

// Direct branch
let isBarrier = 1 in {
  // Short branch
  def JUMP : FBRANCH<0b110100, (outs), (ins jmptarget:$dst),
                   "JUMP \t$dst",
                   [(br bb:$dst)]>;
}



//===----------------------------------------------------------------------===//
// Call Instructions...
//
let isCall = 1 in
  // All calls clobber the non-callee saved registers. SPW is marked as
  // a use to prevent stack-pointer assignments that appear immediately
  // before calls from potentially appearing dead. Uses for argument
  // registers are added manually.
  let Defs = [Sa, Sb,Sc,Sd, STATUS],
      Uses = [STATUS] in {
    def CALL0 : FBRANCH<0x0,
                          (outs), (ins GR8:$dst),
                          "call \t$dst ;call0", 
						  [(Picoblazecall addrcall:$dst)]>;
 
  

    def CALL1 : FBRANCH<0x0,
                          (outs), (ins i16imm:$dst),
                          "call \t$dst ; call1", 
						  [(Picoblazecall addrcall:$dst)]>;

    //def CALL2 : FBRANCH<0x0,
    //                      (outs), (ins GR16:$dst),
    //                      "call \t$dst ;call2", 
	//					  [(Picoblazecall addrcall:$dst)]>;
 
    def CALL3 : FBRANCH<0x0,
                          (outs), (ins i16imm:$dst),
                          "call \t$dst ; call3", 
						  [(Picoblazecall addrcall:$dst)]>;

   def CALL4: FBRANCH<0x0,
                          (outs), (ins GR8:$dst),
                          "call \t$dst ;call4", 
						  [(Picoblazecall  (PicoblazeWrapper  addrcall:$dst) )]>;
  }




//===----------------------------------------------------------------------===//
// Move Instructions

// FIXME: Provide proper encoding!

def LOAD_REG   :   FALUI<0b000001,
                   (outs GR8:$dst), (ins GR8:$src),
                   "LOAD \t{$dst, $src} ;\tLOAD_REG",
                   []>;
def LOAD_REGI   :   FALUI<0b000001,
                   (outs GR8:$dst), (ins i8imm:$src),
                   "LOAD \t{$dst, $src} ;\tLOAD_REGI",
                   [ (set GR8:$dst ,imm:$src)]>;
//}

let  mayLoad=1 in {



def FETCH_I  : FALUI<0b101110,
                   (outs GR8:$dst), (ins  i8imm:$ptr),
                   "FETCH \t{$dst,$ptr} ;\tFETCH_I",
                   [( set GR8:$dst,(load imm:$ptr))]>;

def FETCH_R  : FALUI<0b101110,
                   (outs GR8:$dst), (ins  GR8:$ptr),
                   "FETCH \t{$dst,$ptr} ;\tFETCH_R",
                   [( set GR8:$dst,(load GR8:$ptr))]>;

def FETCH_FRAMEI  : Pseudo<
                   (outs GR8:$dst), (ins  i8imm:$ptr),
                   "FETCH \t{$dst,$ptr} ;\tFETCH_FRAMEI",
                   []>;
}



 let Constraints = "$src = $dst" ,
	 Defs = [STATUS], 
	 Uses = [STATUS] 
	  in {


def ADDC8rr  : FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, GR8:$src2),
                   "ADDCY\t { $dst,$src2}",
                   [(set GR8:$dst,(adde GR8:$src, GR8:$src2)),(implicit STATUS)
                    ]>;
def ADDC8ri  :  FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, i8imm:$src2),
                   "ADDCY \t{ $dst, $src2}",
                   [(set GR8:$dst, (adde GR8:$src, imm:$src2 )),(implicit STATUS)
                    ]>;
def SUBC8rr  : FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, GR8:$src2),
                   "SUBCY\t { $dst,$src2}",
                   [(set GR8:$dst,(sube GR8:$src, GR8:$src2)),(implicit STATUS)
                    ]>;
def SUBC8ri  :  FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, i8imm:$src2),
                   "SUBCY \t{ $dst, $src2}",
                   [(set GR8:$dst, (sube GR8:$src, imm:$src2 )),(implicit STATUS)
                    ]>;

}


let Constraints = "$src = $dst"  in {

def OR8rr  : FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, GR8:$src2),
                   "OR\t { $dst,$src2}",
                   [(set GR8:$dst, (or GR8:$src, GR8:$src2))]>;
def OR8ri  :  FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, i8imm:$src2),
                   "OR\t{ $dst, $src2}",
                   [(set GR8:$dst, (or GR8:$src, imm:$src2))
                   ]>;
}



let Constraints = "$src = $dst" ,

	Defs = [STATUS] 
 in {

def ADD8rr  : FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, GR8:$src2),
                   "ADD\t { $dst,$src2}",
                   [(set GR8:$dst, (addc GR8:$src, GR8:$src2))]>;

def ADD8ri  :  FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, i8imm:$src2),
                   "ADD \t{ $dst, $src2}",
                   [(set GR8:$dst, (addc GR8:$src, imm:$src2))
                   ]>;
def SUB8rr  : FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, GR8:$src2),
                   "SUB\t { $dst,$src2}",
                   [(set GR8:$dst, (subc GR8:$src, GR8:$src2)),
                    (implicit STATUS)]>;

def SUB8ri  :  FALUI<0x0,
                   (outs GR8:$dst), (ins GR8:$src, i8imm:$src2),
                   "SUB \t{ $dst, $src2}",
                   [(set GR8:$dst, (subc GR8:$src, imm:$src2)),
                    (implicit STATUS)]>;
}

def : Pat<(add GR8:$src, GR8:$src2),
          (ADD8rr GR8:$src, GR8:$src2)>;

def : Pat<(sub GR8:$src, GR8:$src2),
          (SUB8rr GR8:$src, GR8:$src2)>;


// FIXME: Provide proper encoding!
let  isAsCheapAsAMove = 1 in {
def LOAD_I     : FALUI<0b000000,
                   (outs GR8:$dst), (ins i8imm:$src),
                   "LOAD\t{$dst, $src}",
                   [(set GR8:$dst, imm:$src)]>;
def LOAD_I2     : FALUI<0b000000,
                   (outs GR8:$dst), (ins i16imm:$src),
                   "LOAD\t{$dst, $src}",
                   []>;
}

let  mayStore=1 in {

def STORE_I  : FALUI<0b101110,
                   (outs), (ins i8imm:$ptr, GR8:$value),
                   "STORE \t{$value,$ptr}",
                   [(store GR8:$value , imm:$ptr)]>;

def STORE_R  : FALUI<0b101111,
                   (outs), (ins GR8:$ptr, GR8:$value),
                   "STORE \t{$value,$ptr}",
                   [(store GR8:$value, GR8:$ptr)]>;

def STORE_FRAMEI  : Pseudo<
                   (outs), (ins i8imm:$ptr , GR8:$value),
                   "STORE \t{$value,$ptr};\tSTORE_FRAMEI",
                   []>;
}







