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
   api("us.ihmc:simulation-construction-set:0.21.9")
   api("us.ihmc:euclid:0.16.2")
   api("us.ihmc:simulation-construction-set-tools:0.13.0-210804-2")
   api("us.ihmc:ihmc-robotics-toolkit:0.13.0-210804-2")
}

