prj_project open "ipodboard.ldf"
prj_run Synthesis -impl impl1 -forceAll
prj_run Translate -impl impl1
prj_run Map -impl impl1
prj_run PAR -impl impl1
prj_run Export -impl impl1 -task Jedecgen
prj_project save
prj_project close
