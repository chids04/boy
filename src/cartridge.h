
#include <cstdint>

enum MBC_Type {
    MBC_None = 0x0,
    MBC_1,
    MBC_1_RAM,
    MBC_1_BATTERY_RAM,
    MBC_2 = 0x05,
    MBC_2_BATTERY_RAM,
    MBC_NONE_RAM = 0x08,
    MBC_NONE_BATTERY_RAM,
} ;

typedef struct {
    MBC_Type mbc_type;
    uint8_t *rom;

} Cartridge;
