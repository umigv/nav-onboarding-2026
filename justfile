default:
    @just --list

# ── Environment ────────────────────────────────────────────────────────────────

# Install dependencies and configure the dev environment
setup:
    pixi run python3 scripts/setup_environment.py

# Regenerate pyrightconfig.json from packages in src/
generate-pyrightconfig:
    pixi run python3 scripts/generate_pyrightconfig.py

# ── Build & Test ───────────────────────────────────────────────────────────────

# Build the entire workspace
build *args:
    pixi run colcon build {{args}}

# Run all tests
test *args:
    pixi run colcon test {{args}}
    pixi run colcon test-result --verbose

# Build a single package and its dependencies
build-pkg pkg *args:
    pixi run colcon build --packages-up-to {{pkg}} {{args}}

# Run tests for a single package
test-pkg pkg *args:
    pixi run colcon test --packages-select {{pkg}} {{args}}
    pixi run colcon test-result --verbose

# Delete build, install, and log directories
clean:
    rm -rf build install log

# ── Code Quality ───────────────────────────────────────────────────────────────

# Auto-format all source files
format *args:
    pixi run python3 scripts/format.py {{args}}

# Run all linters
lint *args:
    pixi run python3 scripts/lint.py {{args}}

# ── Packages ───────────────────────────────────────────────────────────────────

# Create a new ROS 2 package (type: python|cpp)
create-pkg dir pkg type='python':
    python3 scripts/create_package.py {{dir}} {{pkg}} --type {{type}}
    just generate-pyrightconfig

# ── ROS 2 ──────────────────────────────────────────────────────────────────────

# Capture a single message from a topic to a file
extract topic output:
    pixi run python3 scripts/extract_message.py {{topic}} {{output}}

# Publish a message to a topic from a file
publish topic input rate='once':
    pixi run python3 scripts/publish_message.py {{topic}} {{input}} {{rate}}
