# Native SDKs

Run `setup_deps.bat` to install the packages pinned by version and SHA-256 in
`dependencies.json`:

- ovstage 0.2.1.385922.d4f26803
- ovrtx 0.5.0.377615.868bf616
- ovphysx 0.6.4.72118723

ovphysx 0.6.4 requires ovstage 0.2.1. The viewer and physics use the same shared
ovstage loader. The build copies the ovphysx DLLs, plugins, codeless USD schemas,
and ovstage loader beside each executable. Stage and renderer runtimes remain
in the project's `deps` directory.

The installer verifies downloads and refuses to overwrite unrecognized SDK
folders. A recognized previous ovstage 0.2.0 installation is preserved in a
`deps/ovstage-previous-*` backup before upgrading. Nested ovphysx archives are
unpacked to the same `deps/<name>` layout as the other SDKs.

Installed packages retain their licenses and notices. SDK binaries, backups,
and download archives are ignored by Git. No original FBIK checkout is required.
