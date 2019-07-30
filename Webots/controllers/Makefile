TARGETS=soccer.Makefile symmetry.Makefile visual_tracking.Makefile walk.Makefile motion_player.Makefile

.PHONY: release debug profile clean

release debug profile clean: $(TARGETS)

%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)
	+@cp -f Makefile.robotis-op2 $(@:.Makefile=)/
