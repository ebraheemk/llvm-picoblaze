//===-- PicoblazeMCAsmInfo.h - Picoblaze asm properties --------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the PicoblazeMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef PicoblazeTARGETASMINFO_H
#define PicoblazeTARGETASMINFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {
  class StringRef;
  class Target;

  class PicoblazeMCAsmInfo : public MCAsmInfo {
    virtual void anchor();
  public:
    explicit PicoblazeMCAsmInfo(const Target &T, StringRef TT);
  };

} // namespace llvm

#endif
