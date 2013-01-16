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
                                          const TargetRegisterInfo *TRI) const 
{
	PR_FUNCTION();
  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = *MF.getFrameInfo();

  MachineMemOperand *MMO =
    MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(FrameIdx),
                            MachineMemOperand::MOStore,
                            MFI.getObjectSize(FrameIdx),
                            MFI.getObjectAlignment(FrameIdx));

 if (RC == &Picoblaze::GR8RegClass)
   {
	    //  BuildMI(MBB, MI, DL, get(Picoblaze::MOV8mr))
		//   .addFrameIndex(FrameIdx).addImm(0)
		//   .addReg(SrcReg, getKillRegState(isKill)).addMemOperand(MMO);
    }
  else
    llvm_unreachable("Cannot store this register to stack slot!");
}

void PicoblazeInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator MI,
                                           unsigned DestReg, int FrameIdx,
                                           const TargetRegisterClass *RC,
                                           const TargetRegisterInfo *TRI) const
{
	PR_FUNCTION();
	/*
  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = *MF.getFrameInfo();

  MachineMemOperand *MMO =
    MF.getMachineMemOperand(MachinePointerInfo::getFixedStack(FrameIdx),
                            MachineMemOperand::MOLoad,
                            MFI.getObjectSize(FrameIdx),
                            MFI.getObjectAlignment(FrameIdx));

  if (RC == &Picoblaze::GR8RegClass)
    BuildMI(MBB, MI, DL, get(Picoblaze::MOV8rm))
      .addReg(DestReg).addFrameIndex(FrameIdx).addImm(0).addMemOperand(MMO);
  else
    llvm_unreachable("Cannot store this register to stack slot!");
	*/
}

void PicoblazeInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I, DebugLoc DL,
                                  unsigned DestReg, unsigned SrcReg,
                                  bool KillSrc) const 
{
	PR_FUNCTION();
	
  unsigned Opc;
  if (Picoblaze::GR8RegClass.contains(DestReg, SrcReg))
    Opc = Picoblaze::LOAD_REG;
  else
    llvm_unreachable("Impossible reg-to-reg copy");

  BuildMI(MBB, I, DL, get(Opc), DestReg)
    .addReg(SrcReg, getKillRegState(KillSrc));
	
}



