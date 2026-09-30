#!/usr/bin/env bash
# ============================================================
#  build.sh — fast path. Finds what you already have, builds, installs.
#
#    ./build.sh              build debug APK           (seconds, warm daemon)
#    ./build.sh run          build + adb install + launch
#    ./build.sh release      unsigned release build
#    ./build.sh clean        wipe build outputs
#    ./build.sh setup        one-time: install anything actually missing
#    ./build.sh doctor       show what was detected and stop
#
#  Nothing is installed unless you ask for `setup`, and even then only the
#  pieces that are genuinely absent. The Gradle daemon is left running on
#  purpose — that is what makes the second build take seconds.
# ============================================================
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-debug}"
c() { printf '\033[1;36m%s\033[0m\n' "$*"; }
w() { printf '\033[1;33m%s\033[0m\n' "$*"; }
e() { printf '\033[1;31m%s\033[0m\n' "$*" >&2; }

# ---------------- find a JDK 17 or 21 (no install) ----------------
find_java() {
  [ -n "${JAVA_HOME:-}" ] && [ -x "$JAVA_HOME/bin/javac" ] && { echo "$JAVA_HOME"; return; }
  for d in /usr/lib/jvm/java-17-openjdk /usr/lib/jvm/java-21-openjdk \
           /usr/lib/jvm/default /usr/lib/jvm/java-17-openjdk-amd64; do
    [ -x "$d/bin/javac" ] && { echo "$d"; return; }
  done
  # anything on PATH
  command -v javac >/dev/null && {
    echo "$(dirname "$(dirname "$(readlink -f "$(command -v javac)")")")"; return; }
  echo ""
}

# ---------------- find the Android SDK (no install) ----------------
find_sdk() {
  for d in "${ANDROID_SDK_ROOT:-}" "${ANDROID_HOME:-}" \
           "$HOME/Android/sdk" "$HOME/Android/Sdk" /opt/android-sdk; do
    [ -n "$d" ] && [ -d "$d/platforms" ] && { echo "$d"; return; }
  done
  for d in "$HOME/Android/sdk" "$HOME/Android/Sdk"; do
    [ -d "$d" ] && { echo "$d"; return; }      # exists but no platforms yet
  done
  echo ""
}

JDK="$(find_java)"
SDK="$(find_sdk)"
[ -n "$JDK" ] && export JAVA_HOME="$JDK"
[ -n "$SDK" ] && { export ANDROID_SDK_ROOT="$SDK" ANDROID_HOME="$SDK"
                   export PATH="$SDK/platform-tools:$SDK/cmdline-tools/latest/bin:$PATH"; }

PLATFORM="android-34"
HAVE_PLATFORM=0
[ -n "$SDK" ] && [ -d "$SDK/platforms/$PLATFORM" ] && HAVE_PLATFORM=1

# ---------------- doctor ----------------
if [ "$MODE" = "doctor" ]; then
  c "== detected =="
  echo "  JAVA_HOME   ${JDK:-NOT FOUND}"
  [ -n "$JDK" ] && echo "              $("$JDK/bin/java" -version 2>&1 | head -1)"
  echo "  SDK         ${SDK:-NOT FOUND}"
  echo "  $PLATFORM  $([ $HAVE_PLATFORM = 1 ] && echo present || echo MISSING)"
  echo "  gradlew     $([ -x ./gradlew ] && echo present || echo 'missing (setup will make it)')"
  echo "  adb         $(command -v adb || echo 'not on PATH')"
  echo "  daemon      $(pgrep -fc GradleDaemon 2>/dev/null || echo 0) running"
  exit 0
fi

# ---------------- one-time setup ----------------
if [ "$MODE" = "setup" ]; then
  if [ -z "$JDK" ]; then
    c "installing JDK 17"; sudo pacman -S --needed --noconfirm jdk17-openjdk
    sudo archlinux-java set java-17-openjdk 2>/dev/null || true
    JDK="$(find_java)"; export JAVA_HOME="$JDK"
  else c "JDK already present: $JDK"; fi

  SDK="${SDK:-$HOME/Android/sdk}"
  if [ ! -x "$SDK/cmdline-tools/latest/bin/sdkmanager" ]; then
    c "fetching Android command-line tools -> $SDK"
    mkdir -p "$SDK/cmdline-tools"
    curl -fL -o /tmp/cmdline.zip \
      "https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip"
    rm -rf /tmp/cmdline-tools "$SDK/cmdline-tools/latest"
    unzip -q /tmp/cmdline.zip -d /tmp && mv /tmp/cmdline-tools "$SDK/cmdline-tools/latest"
  else c "command-line tools already present"; fi

  export ANDROID_SDK_ROOT="$SDK" ANDROID_HOME="$SDK"
  export PATH="$SDK/cmdline-tools/latest/bin:$SDK/platform-tools:$PATH"
  if [ ! -d "$SDK/platforms/$PLATFORM" ]; then
    c "installing $PLATFORM + build-tools"
    yes | sdkmanager --licenses >/dev/null 2>&1 || true
    sdkmanager --install "platforms;$PLATFORM" "build-tools;34.0.0" "platform-tools" >/dev/null
  else c "$PLATFORM already installed"; fi

  command -v adb >/dev/null || sudo pacman -S --needed --noconfirm android-tools
  c "setup done — from now on just run ./build.sh"
  MODE=debug
fi

# ---------------- preflight ----------------
[ -n "$JDK" ] || { e "no JDK found. Run: ./build.sh setup"; exit 1; }
[ -n "$SDK" ] || { e "no Android SDK found. Run: ./build.sh setup"; exit 1; }
[ "$HAVE_PLATFORM" = 1 ] || [ "$MODE" = "clean" ] || \
  w "$PLATFORM not in $SDK/platforms — if the build fails, run: ./build.sh setup"

# local.properties: only rewrite when it is actually wrong
want="sdk.dir=$SDK"
[ -f local.properties ] && [ "$(cat local.properties)" = "$want" ] || echo "$want" > local.properties

# wrapper: generate once, never again
if [ ! -x ./gradlew ]; then
  c "generating the Gradle wrapper (one time)"
  if command -v gradle >/dev/null; then gradle wrapper --gradle-version 8.9 >/dev/null
  else e "no ./gradlew and no system gradle. Run: sudo pacman -S gradle"; exit 1; fi
fi

# ---------------- build (daemon ON: this is the whole point) ----------------
case "$MODE" in
  clean)   c "cleaning"; ./gradlew clean; exit 0 ;;
  release) c "release build"; ./gradlew assembleRelease
           APK=$(find app/build/outputs/apk/release -name '*.apk' | head -1) ;;
  *)       c "debug build"; ./gradlew assembleDebug
           APK=app/build/outputs/apk/debug/app-debug.apk ;;
esac

[ -f "$APK" ] || { e "no APK produced"; exit 1; }
c "APK: $APK ($(du -h "$APK" | cut -f1))"

# ---------------- install ----------------
if [ "$MODE" = "run" ] || [ "${2:-}" = "run" ] || [ "${1:-}" = "run" ]; then
  command -v adb >/dev/null || { e "adb not installed: sudo pacman -S android-tools"; exit 1; }
  adb devices | grep -qw device || {
    e "no device. Enable USB debugging, plug in, accept the RSA prompt."; exit 1; }
  c "installing"
  adb install -r "$APK"
  adb shell monkey -p in.twopybot.app -c android.intent.category.LAUNCHER 1 >/dev/null 2>&1 || true
  c "launched"
fi
