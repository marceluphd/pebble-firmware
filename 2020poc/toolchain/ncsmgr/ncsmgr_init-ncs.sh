pushdir "${baseDir}/.."

log -n "Checking out NCS ${ncs_tag} ..."
(
    if [ "$1" = "--just-update" ]; then
        cd nrf
        git fetch origin
    else
        west init -m https://github.com/nrfconnect/sdk-nrf --mr "${ncs_tag}"
        cd nrf
    fi

    git checkout "${ncs_tag}"
    cd -
) 2>/dev/null 1>&2
pass

log "Updating repos ..."
west update
pass

west zephyr-export

## including this step enables NCS to use the cmake module, must be tested first
# cmake -P toolchain/cmake/NcsToolchainConfig.cmake

popdir
