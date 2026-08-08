.PHONY: check

check:
	python3 scripts/validate_assets.py
	python3 -m unittest discover -s tests -v
