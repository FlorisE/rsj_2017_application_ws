const { events, Job, Group } = require("brigadier")

const image = "rtmaist.azurecr.io/ros-robot-xenial-moveit:latest"

events.on("push", function(e, project) {
  var getImage = new Job("get-image", image)
  getImage.imageForcePull = true

  var build = new Job("build", image)
  build.tasks = [
    ". /opt/ros/kinetic/setup.sh",
    "cd /src",
    "catkin_make"
  ]

  var test = new Job("test", image)
  test.tasks = [
    ". /opt/ros/kinetic/setup.sh",
    "cd /src",
    "catkin_make run_tests"
  ]

  Group.runEach([getImage, build, test])
})
