set(LLVM_TARGET_DEFINITIONS Picoblaze.td)

tablegen(LLVM PicoblazeGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM PicoblazeGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM PicoblazeGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM PicoblazeGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM PicoblazeGenCallingConv.inc -gen-callingconv)
tablegen(LLVM PicoblazeGenSubtargetInfo.inc -gen-subtarget)
add_public_tablegen_target(PicoblazeCommonTableGen)

add_llvm_target(PicoblazeCodeGen
  PicoblazeBranchSelector.cpp
  PicoblazeISelDAGToDAG.cpp
  PicoblazeISelLowering.cpp
  PicoblazeInstrInfo.cpp
  PicoblazeFrameLowering.cpp
  PicoblazeMachineFunctionInfo.cpp
  PicoblazeRegisterInfo.cpp
  PicoblazeSubtarget.cpp
  PicoblazeTargetMachine.cpp
  PicoblazeSelectionDAGInfo.cpp
  PicoblazeAsmPrinter.cpp
  PicoblazeMCInstLower.cpp
  )

add_dependencies(LLVMPicoblazeCodeGen intrinsics_gen)

add_subdirectory(InstPrinter)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
