add_executable(ranger_nav src/ranger_nav.py)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ranger_nav
  DESTINATION lib/${PROJECT_NAME}
)