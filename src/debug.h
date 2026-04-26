#include "common.h"

// the idea for the debug menu is that each frame, we store a list of these debug frame snapshots,
struct DebugFrame {
  const char* instr_name;
  CPU *cpu_state;

};
