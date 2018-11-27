
all:

tests:
	$(MAKE) -C lib-unicorn tests

tests-clean:
	$(MAKE) -C lib-unicorn tests-clean

