const { events, Job, Group } = require("brigadier")

const image = "ros:kinetic-robot-xenial"

function getTasks(arguments) {
  let tasksHead = [
    "apt-get update",
    ". /opt/ros/kinetic/setup.sh",
    "cd /src",
    "wstool up",
    "rosdep install --from-paths src --ignore-src -r -y"
  ]
  return tasksHead.concat(arguments)
}

events.on("push", function(e, project) {
  var getImage = new Job("get-image", image)
  getImage.imageForcePull = true
  getImage.imagePullSecrets = ["regcred"]

  var build = new Job("build", image)
  build.tasks = getTasks("catkin_make")

  var test = new Job("test", image)
  test.tasks = getTasks("catkin_make run_tests")

  Group.runEach([getImage, build, test])
})
