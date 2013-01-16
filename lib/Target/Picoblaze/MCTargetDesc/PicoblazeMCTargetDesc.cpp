//===-- PicoblazeMCTargetDesc.cpp - Picoblaze Target Descriptions ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Picoblaze specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "PicoblazeMCTargetDesc.h"
#include "PicoblazeMCAsmInfo.h"
#include "InstPrinter/PicoblazeInstPrinter.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "PicoblazeGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "PicoblazeGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "PicoblazeGenRegisterInfo.inc"

using namespace llvm;

static MCInstrInfo *createPicoblazeMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitPicoblazeMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createPicoblazeMCRegisterInfo(StringRef TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitPicoblazeMCRegisterInfo(X, Picoblaze::SP);
  return X;
}

static MCSubtargetInfo *createPicoblazeMCSubtargetInfo(StringRef TT, StringRef CPU,
                                                    StringRef FS) {
  MCSubtargetInfo *X = new MCSubtargetInfo();
  InitPicoblazeMCSubtargetInfo(X, TT, CPU, FS);
  return X;
}

static MCCodeGenInfo *createPicoblazeMCCodeGenInfo(StringRef TT, Reloc::Model RM,
                                                CodeModel::Model CM,
                                                CodeGenOpt::Level OL) {
  MCCodeGenInfo *X = new MCCodeGenInfo();
  X->InitMCCodeGenInfo(RM, CM, OL);
  return X;
}

static MCInstPrinter *createPicoblazeMCInstPrinter(const Target &T,
                                                unsigned SyntaxVariant,
                                                const MCAsmInfo &MAI,
                                                const MCInstrInfo &MII,
                                                const MCRegisterInfo &MRI,
                                                const MCSubtargetInfo &STI) {
  if (SyntaxVariant == 0)
    return new PicoblazeInstPrinter(MAI, MII, MRI);
  return 0;
}

extern "C" void LLVMInitializePicoblazeTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<PicoblazeMCAsmInfo> X(ThePicoblazeTarget);

  // Register the MC codegen info.
  TargetRegistry::RegisterMCCodeGenInfo(ThePicoblazeTarget,
                                        createPicoblazeMCCodeGenInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(ThePicoblazeTarget, createPicoblazeMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(ThePicoblazeTarget,
                                    createPicoblazeMCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(ThePicoblazeTarget,
                                          createPicoblazeMCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(ThePicoblazeTarget,
                                        createPicoblazeMCInstPrinter);
}
