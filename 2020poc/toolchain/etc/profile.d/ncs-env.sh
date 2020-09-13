toWindowsPath () 
{ 
    local path="$*";
    if [ "${path:1:1}" = ":" ]; then
        echo $path;
    else
        local str="${path//\//\\}";
        local drive="${path:1:1}";
        echo "${str/\\?/${drive}\:}";
    fi
}
toolchainRoot="${EXEPATH}"

export PATH=/ncsmgr:/opt/bin:/opt/bin/Scripts:"${PATH}"
export PYTHONPATH=/opt/bin:/opt/bin/Lib:/opt/bin/Lib/site-packages
export GNUARMEMB_TOOLCHAIN_PATH=/opt
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export ZEPHYR_BASE=$(toWindowsPath "$(realpath "${toolchainRoot}/../zephyr")")

# this setting ensures that ln is able to create soft links on windows
export MSYS=winsymlinks:nativestrict

cd "${ZEPHYR_BASE}/.."

if [ ! -f .west/config ] ; then
    echo "NCS repositories are not yet cloned here. To do so run the following command:"
    echo
    echo "ncsmgr init-ncs"
fi
