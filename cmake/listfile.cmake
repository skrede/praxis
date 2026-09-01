function(praxis_listfile_tokens PATH OUT)
    file(READ "${PATH}" text)
    string(REGEX REPLACE "#[^\n]*" "" text "${text}")
    string(REGEX REPLACE ";" "\\\\;" text "${text}")
    string(REGEX REPLACE "\\(" " ( " text "${text}")
    string(REGEX REPLACE "\\)" " ) " text "${text}")
    string(REGEX REPLACE "[ \t\r\n]+" ";" text "${text}")
    set(${OUT} "${text}" PARENT_SCOPE)
endfunction()

# A value the build and a gate must agree on is written once, in the listfile, and read back out
# here rather than copied into the gate where the two could drift. The list wanted is the one whose
# name opens a command's argument list, taken at its first occurrence: a later command propagating
# the same name into another scope repeats it without redeclaring it. The keywords that open a
# set()'s trailing options end the values just as the closing parenthesis does. Nothing but a
# parenthesis is ever stored, because set() reads CACHE and PARENT_SCOPE as its own keywords
# wherever they appear and would take a token carrying one of those spellings as a mode.
function(praxis_listfile_set PATH NAME OUT)
    praxis_listfile_tokens("${PATH}" tokens)

    set(entries "")
    set(collecting FALSE)
    set(opened FALSE)
    foreach (token IN LISTS tokens)
        if (token STREQUAL "")
            continue()
        elseif (collecting AND token MATCHES "^(CACHE|FORCE|PARENT_SCOPE|\\))$")
            break()
        elseif (collecting)
            list(APPEND entries "${token}")
        elseif (opened AND token STREQUAL "${NAME}")
            set(collecting TRUE)
        endif ()
        set(opened FALSE)
        if (token STREQUAL "(")
            set(opened TRUE)
        endif ()
    endforeach ()
    set(${OUT} "${entries}" PARENT_SCOPE)
endfunction()
