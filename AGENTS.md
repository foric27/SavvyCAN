$schema: not-applicable

# SavvyCAN Agent Notes

- Build system is Qt/qmake, not CMake. Main app project is `SavvyCAN.pro`; tests live in a separate qmake project at `test/test.pro`.
- Minimum supported Qt in the main app is Qt `5.14+` (`SavvyCAN.pro` hard-fails on older versions). `README.md` notes a `QT6WIP` branch exists, but this `master` branch is still Qt5-first.
- Normal local build flow is `qmake CONFIG+=release SavvyCAN.pro` then `make` on Linux or `nmake /C` on Windows MSVC. Debug build is `qmake CONFIG+=debug SavvyCAN.pro` then `make`.
- If qmake/build breaks after project file changes, the repo README explicitly recommends: `qmake`, `make clean`, `make`.
- CI source of truth is `.github/workflows/build.yml`. It only builds `master`, and successful pushes publish/update the `continuous` prerelease automatically.
- Windows CI uses MSVC + `qmake` + `nmake`, not MinGW. It also runs `lrelease` manually before compile, with Qt `bin` added to `PATH`.
- Linux CI runs `qmake CONFIG+=release PREFIX=/usr SavvyCAN.pro`, `make -j$(grep -c ^processor /proc/cpuinfo)`, then `lrelease translations/*.ts`, and packages with `linuxdeployqt`.
- Translation files are part of the shipped app/package. When changing `translations/*.ts`, keep CI packaging in mind and make sure generated `.qm` files are still expected by the workflow.
- Help docs are shipped in releases by copying `help/*.md` and `help/images/*`; edits under `help/` affect packaged artifacts directly.
- Key project areas:
  - `connections/`: hardware/network adapters and connection UI/factory
  - `bus_protocols/`: higher-level protocol handlers like ISO-TP / UDS / J1939
  - `dbc/`: DBC parsing/editing UI
  - `re/`: reverse-engineering/analysis windows
  - `test/`: separate Qt test target, currently very small and not mirrored in CI workflow
- The repo already contains non-trivial custom connection work for CAN-Hacker in `connections/carbusconnection.*`; be careful not to assume only stock upstream adapters exist.
- Before touching connection code, inspect `connections/canconnection.h`, `connections/canconfactory.cpp`, `connections/newconnectiondialog.*`, and `connections/connectionwindow.cpp` together; adapter support is wired across all of them.
- `SavvyCAN.pro` currently includes `CONFIG += NO_UNIT_TESTS`; test coverage is not part of the default app build, so app-build success does not validate `test/test.pro`.
- The test qmake file is stale in places (`../canbus.cpp`, `../canbus.h`, `../connections/socketcan.cpp`) compared with current tree layout, so do not assume tests build without adjustment.
- Shell helper scripts under `scripts/` are simple manual helpers for translations; `scripts/README.md` expects executable bits (`chmod +x`) before running them on Unix-like systems.
- Connection types are defined in `connections/canconconst.h` (`CANCon::type` enum: GVRET_SERIAL, KVASER, SERIALBUS, REMOTE, KAYAK, MQTT, LAWICEL, CANSERVER, CANLOGSERVER, CARBUS_HACKER). Adding a new adapter requires updating the enum, the factory (`canconfactory.cpp`), and the dialog (`newconnectiondialog.cpp`).
- DBC class hierarchy is in `dbc/dbc_classes.h`: `DBC_NODE` → `DBC_MESSAGE` → `DBC_SIGNAL` (with multiplexing support). `DBCHandler` owns the parser and tree; `DBCSignalHandler` (nested in `DBC_MESSAGE`) manages signals per message.
- Core data structures live in `can_structs.h` (`CANFrame`, `CANBus`). The main window owns the `CANFrameModel` and `DBCHandler` singletons accessed via `MainWindow::getReference()`.
- UI forms are in `ui/*.ui`; each has a matching `*Window` class in the parent directory. Qt Designer edits go in `ui/`, not inline C++.
- QCustomPlot and qmqtt are vendored directly into the repo tree; do not add system-level copies.
