//===-- PicoblazeMCAsmInfo.cpp - Picoblaze asm properties -----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the PicoblazeMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "PicoblazeMCAsmInfo.h"
#include "llvm/ADT/StringRef.h"
using namespace llvm;

void PicoblazeMCAsmInfo::anchor() { }

PicoblazeMCAsmInfo::PicoblazeMCAsmInfo(const Target &T, StringRef TT) {
  PointerSize = 2;

  PrivateGlobalPrefix = ".L";
  WeakRefDirective ="\t.weak\t";
  PCSymbol=".";
  CommentString = ";";

  AlignmentIsInBytes = false;
  AllowNameToStartWithDigit = true;
  UsesELFSectionDirectiveForBSS = true;
}
