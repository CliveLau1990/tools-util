CC_OPTS = -Wall -Werror
CCS += $(CC_OPTS)

all: bin/rgb565tobmp bin/rgb565toppm bin/bmptorgb565 bin/rgb24tobmp

clean:
	rm -rf bin/

bin/rgb24tobmp: src/rgb24tobmp.c bin
	@$(CC) $(CCS) -o bin/rgb24tobmp src/rgb24tobmp.c -lbmp

bin/rgb565tobmp: src/rgb565tobmp.c bin
	@$(CC) $(CCS) -o bin/rgb565tobmp src/rgb565tobmp.c -lbmp

bin/rgb565toppm: src/rgb565toppm.c bin
	@$(CC) $(CCS) -o bin/rgb565toppm src/rgb565toppm.c && echo "Built rgb565toppm."

bin/bmptorgb565: src/bmptorgb565.c bin
	@$(CC) -o bin/bmptorgb565 src/bmptorgb565.c && echo "Built bmptorgb565."

bin:
	@mkdir bin
