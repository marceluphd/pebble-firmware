listTools() {
    local match='(\d+.\d+(?:.[a-z\d\-]+)?)'

    echo -e "git\n\texpected: ${git_version}\n\treported: $(
        "${baseDir}/mingw64/bin/git" --version | head -1 | grep -Po ${match}
    )"

    echo -e "cmake\n\texpected: ${cmake_version}\n\treported: $(
        "${binDir}/cmake" -version | head -1 | grep -Po ${match}
    )"

    echo -e "python\n\texpected: ${python_version}\n\treported: $(
        "${binDir}/python" -V | head -1 | grep -Po ${match}
    )"

    echo -e "ninja\n\texpected: ${ninja_version}\n\treported: $(
        "${binDir}/ninja" --version | head -1 | grep -Po ${match}
    )"

    echo -e "gperf\n\texpected: ${gperf_version}\n\treported: $(
        "${binDir}/gperf" --version | head -1 | grep -Po ${match}
    )"

    echo -e "gccarm\n\texpected: ${gccarm_version}\n\treported: $(
        "${binDir}/arm-none-eabi-gcc" --version | head -1 | grep -Po ${match} | head -1
    )"
}

listPyModules() {
    local req
    local mod
    local result=0
    while IFS= read -r req ; do
        mod=$(echo "${req}" | grep -Eo "^[^=<>;]+")
        expected=$(echo "${req}" | sed "s/${mod}//")
        reported=$(pip show "${mod}" | grep "^Version" | sed "s/Version: //" 2>/dev/null)
        [ $? -ne 0 ] && result=1
        echo -e "${mod}\n\texpected: ${expected}\n\treported: ${reported}"
    done < "${mgrDir}/combined-requirements.txt"
    return ${result}
}

listTools
listPyModules

exit $?
