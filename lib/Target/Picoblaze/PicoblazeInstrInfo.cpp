//===-- PicoblazeInstrInfo.cpp - Picoblaze Instruction Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Picoblaze implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "PicoblazeInstrInfo.h"
#include "Picoblaze.h"
#include "llvm/Support/raw_ostream.h"
#include "PicoblazeMachineFunctionInfo.h"
#include "PicoblazeTargetMachine.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_CTOR
#include "PicoblazeGenInstrInfo.inc"

using namespace llvm;

PicoblazeInstrInfo::PicoblazeInstrInfo(PicoblazeTargetMachine &tm)
  : PicoblazeGenInstrInfo(Picoblaze::ADJCALLSTACKDOWN, Picoblaze::ADJCALLSTACKUP),
    RI(tm, *this) 
{
	PR_FUNCTION();
}

void PicoblazeInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                             MachineBasicBlock::iterator MI,
                                             unsigned SrcReg, bool isKill, int FrameIdx,
                                             const TargetRegisterClass *RC,
                                             const TargetRegisterInfo *TRI
											 ) const 
{
	PR_FUNCTION();
	llvm::errs()  << "SrcReg:"<<SrcReg <<"   Index:" << FrameIdx << '\n';
	
    DebugLoc DL;
    if (MI != MBB.end()) DL = MI->getDebugLoc();
    MachineFunction &MF = *MBB.getParent();
#ifndef NDEBUG
	MF.dump();
#endif
    MachineFrameInfo &MFI = *MF.getFrameInfo();
	 //   unsigned Align = MFI.getObjectAlignment(MVT::i8);
  //MachineMemOperand *MMO =
  //  MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MVT::i8),
  //                          MachineMemOperand::MOStore,
  //                          MFI.getObjectSize(MVT::i8),
  //                          Align); 
   BuildMI(MBB, MI, DL, get(Picoblaze::STORETOSTACK ));//.addMemOperand(MMO);	
	if (RC == &Picoblaze::GR8RegClass)
   {
	      //BuildMI(MBB, MI, DL, get(Picoblaze::ADD8ri ),Picoblaze::BP)
		  // .addReg(Picoblaze::BP)
		  // .addImm(FrameIdx);
	      BuildMI(MBB, MI, DL, get(Picoblaze::STORE_FRAMEI))
			.addImm(FrameIdx)
			.addReg(SrcReg,getKillRegState(isKill))
			;
	      //BuildMI(MBB, MI, DL, get(Picoblaze::ADD8ri ),Picoblaze::BP)
		   //.addReg(Picoblaze::BP)
		   //.addImm(-FrameIdx);
	     	  
    }
  else
    llvm_unreachable("Cannot store this register to stack slot!");
  BuildMI(MBB, MI, DL, get(Picoblaze::REGXXSTACKEND ));//.addMemOperand(MMO);	
#ifndef NDEBUG
  MF.dump();
#endif

}

void PicoblazeInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator MI,
                                           unsigned DestReg, int FrameIdx,
                                           const TargetRegisterClass *RC,
                                           const TargetRegisterInfo *TRI) const
{
	PR_FUNCTION();
	llvm::errs() << "Src:" <<DestReg<<"  Index:"<< FrameIdx << '\n';
  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();
  MachineFunction &MF = *MBB.getParent();
#ifndef NDEBUG
  MF.dump();
#endif
  MachineFrameInfo &MFI = *MF.getFrameInfo();
  //  unsigned Align = MFI.getObjectAlignment(MVT::i8);
  //MachineMemOperand *MMO =
  //  MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(MVT::i8),
  //                          MachineMemOperand::MOLoad,
  //                          MFI.getObjectSize(MVT::i8),
  //                          Align); 
  BuildMI(MBB, MI, DL, get(Picoblaze::LOADFROMSTACK ));//.addMemOperand(MMO);
  	if (RC == &Picoblaze::GR8RegClass)
   {
		//BuildMI(MBB, MI, DL, get(Picoblaze::ADD8ri ),Picoblaze::BP)
		//	.addReg(Picoblaze::BP)
		//	.addImm(FrameIdx);
		BuildMI(MBB, MI, DL, get(Picoblaze::FETCH_FRAMEI), DestReg)
			.addImm(FrameIdx);
			//.addReg(Picoblaze::BP);
		//BuildMI(MBB, MI, DL, get(Picoblaze::ADD8ri ),Picoblaze::BP)
		//	.addReg(Picoblaze::BP)
		//	.addImm(-FrameIdx);   

	//BuildMI(MBB, MI, DL, get(Picoblaze::FETCH_FRAMEI), DestReg)
	 //BuildMI(MBB, MI, DL, get(Picoblaze::FETCH_I), DestReg)
	//		.addImm(FrameIdx)
			;//.addMemOperand(MMO);		
	    }
  else
    llvm_unreachable("Cannot store this register to stack slot!");
  BuildMI(MBB, MI, DL, get(Picoblaze::REGXXSTACKEND ));//.addMemOperand(MMO);
#ifndef NDEBUG
  MF.dump();
#endif
}

void PicoblazeInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I, DebugLoc DL,
                                  unsigned DestReg, unsigned SrcReg,
                                  bool KillSrc) const 
{
	PR_FUNCTION();
	printf("%d,%d\n",DestReg,SrcReg);
	MachineFunction &MF = *MBB.getParent();
#ifndef NDEBUG
	MF.dump();
#endif
  unsigned Opc;
  if (Picoblaze::GR8RegClass.contains(DestReg, SrcReg))
    Opc = Picoblaze::LOAD_REG;
  else
    llvm_unreachable("Impossible reg-to-reg copy");

  BuildMI(MBB, I, DL, get(Opc), DestReg)
    .addReg(SrcReg, getKillRegState(KillSrc));
#ifndef NDEBUG
	MF.dump();
#endif
}

 unsigned PicoblazeInstrInfo::isLoadFromStackSlot(const MachineInstr *MI,
                                       int &FrameIndex) const 
 {
	 printf("isLoadFromStackSlot=%d\n", MI->getOpcode());
	 if( MI->getOpcode()==Picoblaze::FETCH_FRAMEI)
	 {
		 FrameIndex =MI-> getOperand(1).getImm();
		 return 1;
	 }
	 return 0;
 }

