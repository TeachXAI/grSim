BUILDDIR=build
BUILDTYPE=Release

.PHONY: all build mkbuilddir cmake test install clean clean-all demos

all: build

build: mkbuilddir cmake
	cmake --build $(BUILDDIR) --parallel
	@echo "Binary: $(BUILDDIR)/libgrsim/grsim_run"

mkbuilddir:
	[ -d $(BUILDDIR) ] || mkdir $(BUILDDIR)

cmake: CMakeLists.txt
	cmake -S . -B $(BUILDDIR) -DCMAKE_BUILD_TYPE=$(BUILDTYPE) -DGRSIM_BUILD_TESTS=ON

test: build
	ctest --test-dir $(BUILDDIR)/libgrsim --output-on-failure

install: build
	cmake --install $(BUILDDIR)

demos: build
	@mkdir -p output/logs
	$(BUILDDIR)/libgrsim/grsim_run --config config/default.yaml --duration 8 --mode sync --behavior circle --log-dir output/logs --robots 3
	$(BUILDDIR)/libgrsim/grsim_run --config config/default.yaml --duration 8 --mode sync --behavior square --log-dir output/logs --robots 3
	$(BUILDDIR)/libgrsim/grsim_run --config config/default.yaml --duration 8 --mode async --behavior circle --log-dir output/logs --robots 3
	$(BUILDDIR)/libgrsim/grsim_run --config config/default.yaml --duration 8 --mode async --behavior square --log-dir output/logs --robots 3

clean:
	[ -d $(BUILDDIR) ] && cmake --build $(BUILDDIR) --target clean || true

clean-all:
	rm -rf $(BUILDDIR)
