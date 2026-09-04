ThisBuild / scalaVersion := "2.13.16"

lazy val chiselVersion = "6.7.0"

lazy val root = (project in file("."))
  .settings(
    name := "CPUSTCore",
    Compile / run / mainClass := Some("CPUSTC.GenerateChipLabTop"),
    addCompilerPlugin("org.chipsalliance" % "chisel-plugin" % chiselVersion cross CrossVersion.full),
    Test / parallelExecution := true,
    libraryDependencies ++= Seq(
      "org.chipsalliance" %% "chisel" % chiselVersion,
      "org.typelevel" %% "spire" % "0.18.0",
      "edu.berkeley.cs" %% "chiseltest" % "6.0.0" % Test,
      "org.scalatest" %% "scalatest" % "3.2.19" % Test
    ),
    scalacOptions ++= Seq(
      "-language:reflectiveCalls",
      "-deprecation",
      "-feature",
      "-unchecked",
      "-Xcheckinit",
      "-Ymacro-annotations"
    )
  )
