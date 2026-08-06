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
}
