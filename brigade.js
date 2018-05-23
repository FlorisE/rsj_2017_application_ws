const { events, Job } = require("brigadier")

events.on("push", function(e, project) {
  var job = new Job("echo-in-container", "kinetic-ros-base")
  job.tasks = [
    "echo Test"
  ]

  job.run()
})
