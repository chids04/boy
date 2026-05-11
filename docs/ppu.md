docs for ppu 


### mode 2
- this is the oam scan part
- scans oam for sprites to draw, and stores them in the sprite buffer
- sprites need to meet critera should as vertically be in range of the current scan line, some place say that the xpos is checked but others say that it isnt

### mode 3
- this the stage where the ppu is actually drawing pixels.
- ppu hogs vram here so cpu cant access it, if cpu reads from vram at this time then garbage is returned
- the length of this period is variable,
