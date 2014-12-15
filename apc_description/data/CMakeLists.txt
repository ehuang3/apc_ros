project(apc-data)

install(
    DIRECTORY 
    models/
    DESTINATION
    share/apc/models
    COMPONENT
    amazonpickingchallenge-data
    )

install(
    DIRECTORY 
    scenes/
    DESTINATION
    share/apc/scenes
    COMPONENT
    amazonpickingchallenge-data
    )
