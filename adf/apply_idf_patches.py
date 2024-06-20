from os.path import join, isfile

Import("env")

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-espidf")
patchflag_path = join(FRAMEWORK_DIR, ".patching-done")

# patch file only if we didn't do it before
if not isfile(join(FRAMEWORK_DIR, ".patching-done")):
    #patch_file = "esp-adf/idf_patches/idf_v5.1_freertos.patch"
    patch_file = "idf_v5.2_freertos.patch"

    assert isfile(patch_file)

    env.Execute("(cd %s && patch -p1) < %s" % (FRAMEWORK_DIR, patch_file))
    # env.Execute("touch " + patchflag_path)

    def _touch(path):
        with open(path, "w") as fp:
            fp.write("")

    env.Execute(lambda *args, **kwargs: _touch(patchflag_path))