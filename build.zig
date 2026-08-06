const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // The core: no dependencies, no allocation.
    const m68k = b.addModule("m68k", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    // --- tests -------------------------------------------------------------
    const test_step = b.step("test", "Run unit tests");
    const core_tests = b.addTest(.{ .root_module = m68k });
    test_step.dependOn(&b.addRunArtifact(core_tests).step);

    // Sequence-level tests, kept out of core.zig: they drive the public API
    // over many instructions rather than testing one function.
    const system_tests = b.addTest(.{ .root_module = b.createModule(.{
        .root_source_file = b.path("src/system_test.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "m68k", .module = m68k }},
    }) });
    test_step.dependOn(&b.addRunArtifact(system_tests).step);

    // `zig build fuzz --fuzz` searches; plain `zig build test` runs it once on
    // a fixed input, which is enough to keep it compiling and honest.
    const fuzz_tests = b.addTest(.{ .root_module = b.createModule(.{
        .root_source_file = b.path("src/fuzz_test.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "m68k", .module = m68k }},
    }) });
    const fuzz_run = &b.addRunArtifact(fuzz_tests).step;
    test_step.dependOn(fuzz_run);
    b.step("fuzz", "Fuzz step() against a contract-checking bus (add --fuzz)")
        .dependOn(fuzz_run);

    // --- SingleStepTests conformance harness --------------------------------
    const harness_mod = b.createModule(.{
        .root_source_file = b.path("src/harness.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "m68k", .module = m68k }},
    });

    const harness_tests = b.addTest(.{ .root_module = harness_mod });
    test_step.dependOn(&b.addRunArtifact(harness_tests).step);

    const sst = b.addExecutable(.{ .name = "z68k-sst", .root_module = harness_mod });
    b.installArtifact(sst);

    const sst_run = b.addRunArtifact(sst);
    sst_run.setCwd(b.path(".")); // testdata/ is resolved relative to the project
    sst_run.step.dependOn(b.getInstallStep());
    if (b.args) |args| sst_run.addArgs(args);
    b.step("sst", "Run the SingleStepTests conformance suite (needs testdata/)")
        .dependOn(&sst_run.step);

    // The VDP has no dependency on raylib, so its own tests ride along here.
    const vdp_tests = b.addTest(.{ .root_module = b.createModule(.{
        .root_source_file = b.path("examples/genesis_vdp.zig"),
        .target = target,
        .optimize = optimize,
    }) });
    test_step.dependOn(&b.addRunArtifact(vdp_tests).step);

    // --- examples ------------------------------------------------------------
    const amiga = b.addExecutable(.{ .name = "z68k-amiga", .root_module = b.createModule(.{
        .root_source_file = b.path("examples/amiga.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "m68k", .module = m68k }},
    }) });
    b.installArtifact(amiga);

    const amiga_run = b.addRunArtifact(amiga);
    amiga_run.setCwd(b.path(".")); // roms/ is resolved relative to the project
    amiga_run.step.dependOn(b.getInstallStep());
    if (b.args) |args| amiga_run.addArgs(args);
    b.step("amiga", "Boot an Amiga ROM (needs roms/; see examples/amiga.zig)")
        .dependOn(&amiga_run.step);

    // The Genesis example draws with raylib, the only dependency in the tree.
    // It hangs off its own step and is never installed, so nothing else builds
    // or links it. The dependency is lazy, but the build runner still fetches
    // it once for any build after a fresh clone: a build script cannot see
    // which step was asked for, so the call above happens either way.
    const genesis_step = b.step("genesis", "Run a Genesis ROM (needs roms/ and raylib)");
    if (b.lazyDependency("raylib", .{
        .target = target,
        .optimize = optimize,
        .raudio = false, // no sound emulation here, so nothing to play it with
        .rmodels = false,
    })) |raylib_dep| {
        const genesis = b.addExecutable(.{
            .name = "z68k-genesis",
            .root_module = b.createModule(.{
                .root_source_file = b.path("examples/genesis.zig"),
                .target = target,
                .optimize = optimize,
                .link_libc = true, // raylib.h comes in through @cImport
                .imports = &.{.{ .name = "m68k", .module = m68k }},
            }),
        });
        genesis.root_module.linkLibrary(raylib_dep.artifact("raylib"));

        const genesis_run = b.addRunArtifact(genesis);
        genesis_run.setCwd(b.path("."));
        if (b.args) |args| genesis_run.addArgs(args);
        genesis_step.dependOn(&genesis_run.step);
    }
}
