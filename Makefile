include config.mk
PREFIX="/usr/local"
PROJECT="slam-cpp"

default: libslam-cpp tests done

mkdirs:
	@mkdir -p $(LIB_DIR)
	@mkdir -p $(LIB_BUILD_DIR)
	@mkdir -p $(TESTS_BUILD_DIR)
	@mkdir -p $(TESTS_BIN_DIR)

rmdirs:
	@rm -rf $(BUILD_DIR)

deps:
	@sh scripts/dependencies.sh

libslam-cpp: mkdirs
	@make -s -C src

tests: mkdirs
	@make -s -C tests

debug: mkdirs
	@make -C src
	@make -C tests

run_tests: libsr tests
	@sh scripts/test_runner.sh

install:
	@echo "installing $(PROJECT) to [$(PREFIX)/lib]"
	@cp .$(LIB_DIR)/lib$(PROJECT).a $(PREFIX)/lib
	@mkdir -p $(PREFIX)/include/$(PROJECT)
	@cp -r include/* $(PREFIX)/include/$(PROJECT)/
	@echo "$(PROJECT) installed!"

uninstall:
	@echo "removing $(PROJECT)"
	@rm $(PREFIX)/lib/lib$(PROJECT).a
	@rm -rf $(PREFIX)/include/$(PROJECT)

clean: rmdirs
	@echo "cleaning ..."
	@echo "done! :)"

done:
	@echo "done! :)"
