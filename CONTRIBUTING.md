# Contributing

Every change to version pins, messages, the bridge's mappings or the bridged topic list
goes into `CHANGELOG.md`, under "Unreleased", with the reason.

## Making a release

The images on ghcr.io are built only for a release, so each release has images of its own
and a checkout of it always gets the right ones. For a release `1.0.1`:

1. `CHANGELOG.md`: rename "Unreleased" to "1.0.1 (date)" and start a new, empty
   "Unreleased" above it.
2. `compose.yaml`: replace the tag `1.0.0` with `1.0.1` in the `image:` lines;
   `scripts/robot_check.sh` reads the image names from there. The same number goes into
   `<version>` of `baxter_interface_ros2/package.xml` and `moveit_config/package.xml`, and
   `version=` of `baxter_interface_ros2/setup.py`. The message packages and
   `robot_model/package.xml` keep the versions of Rethink's and Dataspeed's originals.
3. Commit, merge into `main`, then tag and push the tag:

   ```bash
   git tag v1.0.1
   git push origin v1.0.1
   ```

4. GitHub Actions (`.github/workflows/images.yml`) builds all images and publishes them as
   `1.0.1` and `latest`. The bridge takes up to 45 minutes when nothing is cached. Until
   the run is done, `docker compose pull` on the new version fails.
5. On GitHub, make a release from the tag with the CHANGELOG's text.

Images for testing, without a release: GitHub -> Actions -> images -> Run workflow, on a
branch. They get the branch's name as their tag (`feature/x` becomes `feature-x`); change
the tag in `compose.yaml` on that branch to use them.
