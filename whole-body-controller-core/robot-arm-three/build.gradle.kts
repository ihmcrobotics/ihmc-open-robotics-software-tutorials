plugins {
   id("us.ihmc.ihmc-build")
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
   api("us.ihmc:simulation-construction-set:0.22.10")
   api("us.ihmc:euclid:0.19.1")
   api("us.ihmc:simulation-construction-set-tools:0.14.0-230207")
   api("us.ihmc:ihmc-simulation-toolkit:0.14.0-230207")
   api("us.ihmc:ihmc-common-walking-control-modules:0.14.0-230207")
   api("us.ihmc:robot-arm-one:source")
   api("us.ihmc:robot-arm-two:source")
}

