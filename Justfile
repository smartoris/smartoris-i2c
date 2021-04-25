test_features := 'drone-stm32-map/gpio'
target := `drone print target 2>/dev/null || echo ""`

# Install dependencies
deps:
	type cargo-readme >/dev/null || cargo +stable install cargo-readme
	type drone >/dev/null || cargo install drone
	rustup target add $(drone print target)

# Reformat the source code
fmt:
	cargo fmt

# Check the source code for mistakes
lint:
	cargo clippy

# Build the documentation
doc:
	cargo doc

# Open the documentation in a browser
doc-open: doc
	cargo doc --open

# Run the tests
test:
	cargo test --features "{{test_features}} std" \
		--target=$(rustc --version --verbose | sed -n '/host/{s/.*: //;p}')

# Update README.md
readme:
	cargo readme -o README.md
