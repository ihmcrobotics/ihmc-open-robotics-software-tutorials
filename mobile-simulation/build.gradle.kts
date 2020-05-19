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
}
