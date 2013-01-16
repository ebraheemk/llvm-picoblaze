add_llvm_library(LLVMPicoblazeDesc
  PicoblazeMCTargetDesc.cpp
  PicoblazeMCAsmInfo.cpp
  )

add_dependencies(LLVMPicoblazeDesc PicoblazeCommonTableGen)
