plugins {
   id("us.ihmc.ihmc-build") version "0.20.2"
}

ihmc {
   group = "us.ihmc"
   version = "0.0"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-open-robotics-software-tutorials"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:simulation-construction-set:0.16.0")
   api("us.ihmc:euclid:0.14.1")
   api("us.ihmc:simulation-construction-set-tools:0.13.0-200518")
   api("us.ihmc:ihmc-simulation-toolkit:0.13.0-200518")
   api("us.ihmc:ihmc-common-walking-control-modules:0.13.0-200518")
   api("us.ihmc:robot-arm-one:source")
   api("us.ihmc:robot-arm-two:source")
}

