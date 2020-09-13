from kconfiglib import Kconfig, \
                       Symbol, Choice, MENU, COMMENT, \
                       BOOL, TRISTATE, STRING, INT, HEX, UNKNOWN, \
                       expr_value, \
                       TRI_TO_STR, TYPE_TO_STR, \
                       standard_config_filename
import sys

anonymousNodeToName = {}
anonymousNameToNode = {}

def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

def _shown_nodes(menu):
    # Returns the list of menu nodes from 'menu' (see _parent_menu()) that
    # would be shown when entering it

    def rec(node):
        res = []

        while node:
            # If a node has children but doesn't have the is_menuconfig flag
            # set, the children come from a submenu created implicitly from
            # dependencies, and are shown (indented) in the same menu as the
            # parent node
            shown_children = \
                rec(node.list) if node.list and not node.is_menuconfig else []

            # Always show the node if it is the root of an implicit submenu
            # with visible items, even when the node itself is invisible. This
            # can happen e.g. if the symbol has an optional prompt
            # ('prompt "foo" if COND') that is currently invisible.
            if shown(node) or shown_children:
                res.append(node)

            res.extend(shown_children)

            node = node.next

        return res

    def shown(node):
        # Show the node if its prompt is visible. For menus, also check
        # 'visible if'. In show-all mode, show everything.
        return _show_all or \
            (node.prompt and expr_value(node.prompt[1]) and not
             (node.item == MENU and not expr_value(node.visibility)))

    if isinstance(menu.item, Choice):
        # For named choices defined in multiple locations, entering the choice
        # at a particular menu node would normally only show the choice symbols
        # defined there (because that's what the MenuNode tree looks like).
        #
        # That might look confusing, and makes extending choices by defining
        # them in multiple locations less useful. Instead, gather all the child
        # menu nodes for all the choices whenever a choice is entered. That
        # makes all choice symbols visible at all locations.
        #
        # Choices can contain non-symbol items (people do all sorts of weird
        # stuff with them), hence the generality here. We really need to
        # preserve the menu tree at each choice location.
        #
        # Note: Named choices are pretty broken in the C tools, and this is
        # super obscure, so you probably won't find much that relies on this.

        # Do some additional work to avoid listing choice symbols twice if all
        # or part of the choice is copied in multiple locations (e.g. by
        # including some Kconfig file multiple times). We give the prompts at
        # the current location precedence.
        seen_syms = {node.item for node in rec(menu.list)
                     if isinstance(node.item, Symbol)}
        res = []
        for choice_node in menu.item.nodes:
            for node in rec(choice_node.list):
                # 'choice_node is menu' checks if we're dealing with the
                # current location
                if node.item not in seen_syms or choice_node is menu:
                    res.append(node)
                    if isinstance(node.item, Symbol):
                        seen_syms.add(node.item)
        return res

    return rec(menu.list)

def _check_valid(sym, s):
    # Returns True if the string 's' is a well-formed value for 'sym'.
    # Otherwise, displays an error and returns False.

    if sym.orig_type not in (INT, HEX):
        # Anything goes for non-int/hex symbols
        return True

    base = 10 if sym.orig_type == INT else 16
    try:
        int(s, base)
    except ValueError:
        eprint("'{}' is a malformed {} value"
               .format(s, TYPE_TO_STR[sym.orig_type]))
        return False

    for low_sym, high_sym, cond in sym.ranges:
        if expr_value(cond):
            low = int(low_sym.str_value, base)
            val = int(s, base)
            high = int(high_sym.str_value, base)

            if not low <= val <= high:
                eprint("{} is outside the range {}-{}"
                       .format(s, low_sym.str_value, high_sym.str_value))

                return False

            break

    return True

def _range_info(sym):
    # Returns a string with information about the valid range for the symbol
    # 'sym', or None if 'sym' doesn't have a range

    if sym.orig_type in (INT, HEX):
        for low, high, cond in sym.ranges:
            if expr_value(cond):
                return "Range: {}-{}".format(low.str_value, high.str_value)

    return None

def _set_value(sym, val):
    if _check_valid(sym, val):
        sym.set_value(val)

def get_anonymous_name(node):
      if node.item in anonymousNodeToName:
          nameString = anonymousNodeToName[node.item]
      else:
          nameString = "__ANON" + str(len(anonymousNameToNode))
          anonymousNameToNode[nameString] = node.item
          anonymousNodeToName[node.item] = nameString
      return nameString

def output_json_tree(list, first):
    for node in list:
        typeString = ""
        valueString = ""
        nameString = ""
        promptString = ""
        assignableString = ""
        isMenu = False

        if hasattr(node.item, "name"):
            nameString = str(node.item.name)

        if node.prompt:
            promptString = node.prompt[0]
        else:
            promptString = nameString
       
        if promptString != "":
            if node.item == MENU:
                typeString = "menu"
                isMenu = True
            elif node.item == COMMENT:
                typeString = "comment"
            elif isinstance(node.item, Symbol) and node.item.choice and node.item.visibility == 2:
                typeString = "choice"
                if node.item.choice.selection is node.item:
                    valueString = "1"
                else:
                    valueString = "0"
            else:
                sc = node.item
                if sc.type == BOOL:
                    isMenu = node.is_menuconfig
                    if node.is_menuconfig and len(sc.assignable) <= 1:
                        assignableString = "0"
                    else:
                        assignableString = "1"
                        if nameString == "None":
                            nameString = get_anonymous_name(node)
                    typeString = "bool"
                    if sc.tri_value > 0:
                        valueString = "1"
                    else:
                        valueString = "0"
                elif sc.type == TRISTATE:
                    typeString = "tristate"
                    eprint("TRISTATE")
                elif sc.type == STRING:
                    typeString = "string"
                    valueString = node.item.str_value.replace('"', '\\"')
                elif sc.type == INT:
                    typeString = "int"
                    valueString = node.item.str_value
                elif sc.type == HEX:
                    typeString = "hex"
                    valueString = node.item.str_value
                elif sc.type == UNKNOWN:
                    typeString = "unknown"

        if typeString != "":
            if first:
                first = False
            else:
                sys.stdout.write(',')
            sys.stdout.write('{"type":"' + typeString + '"')
            if nameString != "" and nameString != "None":
                sys.stdout.write(',"name":"' + nameString + '"')
            if promptString != "":
                sys.stdout.write(',"prompt":"' + promptString.replace('"', '&quot;').replace('\\', '\\\\') + '"')
            if valueString != "":
                sys.stdout.write(',"value":"' + valueString + '"')
            if (node.prompt and expr_value(node.prompt[1]) and not (node.item == MENU and not expr_value(node.visibility))):
                sys.stdout.write(',"shown":1')
            else:
                sys.stdout.write(',"shown":0')
            if assignableString != "":
                sys.stdout.write(',"assignable":' + assignableString)
            if hasattr(node, "help"):
                helpString = str(node.help)
                if helpString != "None":
                    sys.stdout.write(',"help":"' + helpString.replace('\n', ' ').replace('"', '\'').replace('\\', '\\\\') + '"')
            if hasattr(node, "filename"):
                symbolInfoString = str(node.filename) + ":" + str(node.linenr)
                sys.stdout.write(',"info":"' + symbolInfoString.replace('\\', '\\\\') + '"')
            if isMenu:
                sys.stdout.write(',"menu":1')
                children = _shown_nodes(node)
                if (len(children) > 0):
                    sys.stdout.write(',"children":[')
                    output_json_tree(children, True)
                    sys.stdout.write(']')
            else:
                sys.stdout.write(',"menu":0')

            sys.stdout.write('}')

if __name__ == "__main__":
    kconf = Kconfig(sys.argv[1])
    kconf.disable_warnings()
    kconf.load_config(standard_config_filename())
    _show_all = True

    sys.stdout.write("config_filename:" + standard_config_filename() + "\n")

    running = True
    while running:
        cmd = input().strip()
        if cmd == "get_json_tree":
            sys.stdout.write("json_tree:")
            sys.stdout.write('[')
            output_json_tree(_shown_nodes(kconf.top_node), True)
            sys.stdout.write(']')
            sys.stdout.write('\n')
        elif cmd.startswith("set_value "):
            cmd = cmd[10:]
            pos = cmd.find(' ')
            if pos > 0:
                var = cmd[:pos]
                val = cmd[pos + 1:]
                if var in kconf.named_choices:
                    kconf.named_choices[var].set_value(val)
                elif var in kconf.syms:
                    _set_value(kconf.syms[var], val)
                elif var in anonymousNameToNode:
                    _set_value(anonymousNameToNode[var], val)
                else:
                    eprint("ERROR: " +  var + " not in named choices or syms")
        elif cmd.startswith("load_config "):
            anonymousNameToNode.clear()
            anonymousNodeToName.clear()
            filepath = cmd[12:]
            try:
                kconf.load_config(filepath)
            except OSError as e:
                eprint("ERROR: load failed (" + e.strerror + ")")
        elif cmd.startswith("write_config "):
            filepath = cmd[13:]
            kconf.write_config(filepath)
        elif cmd == "quit":
            running = False
        else:
            eprint("ERROR: unknown command " + cmd)
