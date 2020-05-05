# Install dependencies
deps:
	rustup component add clippy
	rustup component add rustfmt
	type cargo-readme >/dev/null || cargo +stable install cargo-readme

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
	cargo test --features std

# Update README.md
readme:
	cargo readme -o README.md
