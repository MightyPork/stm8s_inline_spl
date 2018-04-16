# stm8s_inline_spl

Inlined STM8S SPL (STM8S103) for use with SDCC

This is probably against the license, so use at own risk.

SDCC does not support --gc-sections, so we have to either use #ifdef's 
and compile switches to remove unused code, or inline it.

Here I inlined all of the SPL that's useful for stm8s103 (except the flash file -
there's lots of stuff I don't understand, so you'll do better just keeping it
intact or re-implementing it).

This is intended to be used with SDCC on linux.

**The CMakeLists file is for CLion to stop whining about missing includes and fake syntax errors.
It's not used for building. Remove the .clion suffix if you want to use it**
