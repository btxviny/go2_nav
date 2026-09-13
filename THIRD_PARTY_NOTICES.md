# Third-party notices

This project's own original work is MIT-licensed (see [LICENSE](LICENSE)). It builds on
third-party code and assets under different, separate terms, documented here plainly
rather than glossed over.

## src/go2_ros2_sim_py/ -- no upstream license

Vendored from [go2_ros2_sim_py](https://github.com/abutalipovvv/go2_ros2_sim_py) by
abutalipovvv (commit `92d6bc9`), with our own patch on top (commit history preserved via
`git subtree` -- `git log -- src/go2_ros2_sim_py` shows all of it; see the README's
"Patches to upstream" section for what changed and why).

**That upstream repository has no LICENSE file anywhere, and its README does not address
licensing.** This is a real gap, not an oversight on our part: under default copyright law,
"no license" does not mean "public domain" or "do whatever you want" -- it means the author
retains all rights, and redistribution/modification technically requires their permission
even though the source is public on GitHub. This project uses and modifies that code in
good faith (a public repo, no stated restriction, common practice in the ROS community for
research/hobby robotics code), but if you plan to redistribute this project further, you
may want to reach out to the original author for explicit clarification first.

## src/kiss-icp/ -- MIT

Git submodule, unpatched, pointing directly at
[PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp), which is MIT-licensed (see its own
`LICENSE` file). No conflict with this project's own MIT license.

## blender/assets/ -- CC0 (Poly Haven)

Not committed to this repository (see `.gitignore` and `blender/README.md` for why and how
to re-obtain them) -- but worth noting for completeness: the five props referenced by
`blender/build_office.py` (chess set, baseball, rubber duck, school chair, display
shelves) are [Poly Haven](https://polyhaven.com) assets, released under
[CC0](https://creativecommons.org/publicdomain/zero/1.0/) (public domain, no attribution
legally required, though Poly Haven appreciates it).

## A note on what used to be here

An earlier version of this project also vendored
[spark-fast-lio](https://github.com/MIT-SPARK/spark-fast-lio) (GPLv2-licensed). That
dependency has since been removed entirely (see the README's "FAST-LIO, briefly" section)
-- it's no longer part of this repository, so its GPLv2 terms are no longer a
consideration here.
