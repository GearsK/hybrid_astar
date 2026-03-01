# Fix: gz-common build failure (libz.tbd missing on macOS)

## Problems

1. **gz-common** fails with:
   ```text
   gmake[2]: *** No rule to make target '/Library/Developer/CommandLineTools/SDKs/MacOSX26.sdk/usr/lib/libz.tbd', needed by 'lib/libgz-common-graphics.7.1.0.dylib'.  Stop.
   ```
   gz-common does **not** use the standard CMake `ZLIB_*` variables, so `-DZLIB_LIBRARY` etc. have no effect. The linker is still given the SDK’s `libz.tbd`, which is missing.

2. **gz-utils** can fail with:
   ```text
   install_name_tool: no LC_RPATH load command with path: /opt/homebrew/lib found in: .../libgz-utils-log.4.0.0.dylib, required for specified option "-delete_rpath /opt/homebrew/lib"
   ```
   This happens when the install step tries to strip an RPATH that isn’t in the current dylib (e.g. after a partial or stale install).

## Fix (recommended)

Use Homebrew’s zlib and make the **entire build** see it via **environment variables**, then do a **full clean** and rebuild. That way every package (including gz-common and its deps) that looks for zlib gets Homebrew’s, and there are no stale installs.

### 1. Install zlib via Homebrew

```bash
brew install zlib
```

### 2. Full clean (required)

From your colcon workspace (e.g. `~/workspace`):

```bash
cd /Users/jeffry/workspace   # or your workspace path
rm -rf build install log
```

This avoids the gz-utils `install_name_tool` error and ensures gz-common reconfigures with the right zlib.

### 3. Set env so the build uses Homebrew’s zlib (Apple Silicon)

In the **same shell** where you run `colcon build`:

```bash
export LDFLAGS="-L/opt/homebrew/opt/zlib/lib"
export CPPFLAGS="-I/opt/homebrew/opt/zlib/include"
export PKG_CONFIG_PATH="/opt/homebrew/opt/zlib/lib/pkgconfig:${PKG_CONFIG_PATH}"
```

**Intel Mac** use:

```bash
export LDFLAGS="-L/usr/local/opt/zlib/lib"
export CPPFLAGS="-I/usr/local/opt/zlib/include"
export PKG_CONFIG_PATH="/usr/local/opt/zlib/lib/pkgconfig:${PKG_CONFIG_PATH}"
```

### 4. Rebuild

```bash
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

To make the env vars persistent for that terminal session you can do steps 3 and 4 in one block:

**Apple Silicon:**

```bash
export LDFLAGS="-L/opt/homebrew/opt/zlib/lib"
export CPPFLAGS="-I/opt/homebrew/opt/zlib/include"
export PKG_CONFIG_PATH="/opt/homebrew/opt/zlib/lib/pkgconfig:${PKG_CONFIG_PATH}"
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

**Intel:**

```bash
export LDFLAGS="-L/usr/local/opt/zlib/lib"
export CPPFLAGS="-I/usr/local/opt/zlib/include"
export PKG_CONFIG_PATH="/usr/local/opt/zlib/lib/pkgconfig:${PKG_CONFIG_PATH}"
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

## If it still fails: use an SDK that has libz.tbd

The error can persist because the **current SDK** (e.g. `MacOSX26.sdk`) has **no** `libz.tbd`. Env vars don’t change that; the toolchain still targets that SDK’s `usr/lib`. Fix by pointing the build at an SDK that **does** include `libz.tbd`.

### 1. See which SDKs you have and which have libz

```bash
ls /Library/Developer/CommandLineTools/SDKs/
ls /Library/Developer/CommandLineTools/SDKs/MacOSX15.sdk/usr/lib/libz.tbd 2>/dev/null && echo "MacOSX15 has libz"
ls /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/lib/libz.tbd 2>/dev/null && echo "MacOSX has libz"
```

### 2. Full clean, then build with that SDK

Pick an SDK that has `libz.tbd` (e.g. `MacOSX15.sdk` or `MacOSX.sdk`). In your workspace:

```bash
cd /Users/jeffry/workspace
rm -rf build install log
```

Then build with that SDK (use the path that exists on your machine):

**Option A – use MacOSX15.sdk (or MacOSX14.sdk, etc.):**

```bash
colcon build --cmake-args \
  -DBUILD_TESTING=OFF \
  -DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX15.sdk \
  --merge-install
```

**Option B – use default SDK from xcrun (if it’s not MacOSX26):**

```bash
export SDKROOT=$(xcrun --sdk macosx --show-sdk-path)
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

If `xcrun --show-sdk-path` prints a path that **does** contain `libz.tbd`, Option B is enough. If it prints `MacOSX26.sdk` and that SDK has no libz, use Option A with a specific SDK path that has `libz.tbd`.

### 3. If only MacOSX26.sdk exists and has no libz (e.g. beta OS)

Create a `libz.tbd` stub from Homebrew’s zlib and install it into the SDK (requires sudo):

```bash
brew install zlib
mkdir -p /tmp/tbd
tapi stubify /opt/homebrew/opt/zlib/lib/libz.dylib -o /tmp/tbd/libz.tbd
sudo cp /tmp/tbd/libz.tbd /Library/Developer/CommandLineTools/SDKs/MacOSX26.sdk/usr/lib/
```

Then do a full clean and rebuild **without** changing `CMAKE_OSX_SYSROOT`.

## If you need RPATH / install name

After a successful build, you can add:

```bash
-DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib
```
to your `colcon build --cmake-args ...` line.

## Summary

- **Cause:** The build uses an SDK (e.g. `MacOSX26.sdk`) whose `usr/lib` has no `libz.tbd`. gz-common doesn’t use `ZLIB_*` CMake variables, so env vars alone don’t fix it.
- **Fix (preferred):** Do a full clean and pass **`-DCMAKE_OSX_SYSROOT=...`** to an SDK that **has** `libz.tbd` (e.g. `MacOSX15.sdk` or `MacOSX.sdk`).
- **Fix (if only MacOSX26):** Create `libz.tbd` with `tapi stubify` from Homebrew’s zlib and copy it into the SDK’s `usr/lib`.

---

## `gz sim` not available (only `help` and `plugin` show)

If `gz` runs but **`gz sim`** does not appear in the command list, the Sim binary/plugin is not visible to the `gz` CLI.

### Source (colcon) install

1. **Source your workspace** in the same shell where you run `gz`:
   ```bash
   source /path/to/your/workspace/install/setup.zsh   # or setup.bash
   ```
   Replace `/path/to/your/workspace` with your colcon workspace (e.g. the one where you ran `colcon build`). The `gz` executable discovers commands from the install tree; without sourcing, `gz sim` may not be found.

2. **Confirm gz-sim is built and installed:** Ensure your vcstool/colcon set includes **gz-sim** and that the build (and install) completed successfully. If gz-sim was never built, add it to your repo collection and rebuild.

3. Then run (per [Getting Started — macOS](https://gazebosim.org/docs/latest/getstarted/#macos)):
   ```bash
   # Terminal 1: server
   gz sim -v 4 shapes.sdf -s

   # Terminal 2: GUI (optional)
   gz sim -v 4 -g
   ```

### Binary install (no colcon)

To get **`gz sim`** without building from source:

```bash
brew tap osrf/simulation
brew install gz-jetty   # or gz-harmonic, gz-fortress — see gazebosim.org/docs/latest/install_osx
```

Then run `gz sim shapes.sdf -s` as in the docs. No need to source a workspace.

**If `brew install gz-jetty` succeeded but `gz sim` still only shows `help` and `plugin`:** your shell is using a different `gz` (e.g. from a conda env or a colcon install) that is earlier on `PATH`. Use Homebrew’s `gz` instead:

```bash
# Prefer Homebrew (Apple Silicon)
export PATH="/opt/homebrew/bin:$PATH"
# Intel Mac: export PATH="/usr/local/bin:$PATH"

which gz   # should show /opt/homebrew/bin/gz (or /usr/local/bin/gz)
gz sim -v 4 shapes.sdf -s
```

Or run Gazebo in a shell where the conda env is **not** activated (`conda deactivate`), so `gz` resolves to Homebrew.
