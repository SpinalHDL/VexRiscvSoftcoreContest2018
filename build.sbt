lazy val root = (project in file(".")).
  settings(
    inThisBuild(List(
      organization := "com.github.spinalhdl",
      scalaVersion := "2.11.6",
      version      := "1.0.0"
    )),
    libraryDependencies ++= Seq(
        "org.scalatest" % "scalatest_2.11" % "2.2.1",
        "org.yaml" % "snakeyaml" % "1.8"
    ),
    name := "riscvSoftcoreContest",
    scalaSource in Compile := baseDirectory.value / "hardware" / "scala"
  ).dependsOn(spinalHdlSim, spinalHdlCore, spinalHdlLib, vexRiscv)
lazy val spinalHdlSim = ProjectRef(file("../SpinalHDL"), "SpinalHDL-sim")
lazy val spinalHdlCore = ProjectRef(file("../SpinalHDL"), "SpinalHDL-core")
lazy val spinalHdlLib = ProjectRef(file("../SpinalHDL"), "SpinalHDL-lib")
lazy val vexRiscv = RootProject(file("/home/spinalvm/hdl/VexRiscv"))

addCompilerPlugin("org.scala-lang.plugins" % "scala-continuations-plugin_2.11.6" % "1.0.2")
scalacOptions += "-P:continuations:enable"
fork := true
