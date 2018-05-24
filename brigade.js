const { events, Job } = require("brigadier")

console.log("running brigade")

events.on("push", function(e, project) {
  var job = new Job("build", "ros:kinetic-robot-xenial")
  job.tasks = [
    "apt-get update",
    "apt-get install -y ros-kinetic-moveit-*",
    "/bin/bash",
    ". /opt/ros/kinetic/setup.sh",
    "cd /src"
  ]

  job.run()
})
