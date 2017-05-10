#!/usr/bin/env python3
import os
import sys

from jinja2 import Template
import CppHeaderParser


def walkdir(path):
    files = []
    for (dirpath, dirnames, filenames) in os.walk(path):
        for filename in filenames:
            if filename.endswith('.hpp'):
                files.append(os.sep.join([dirpath, filename]))
    files.reverse()
    return files


def clean_name(name):
    name = name.replace(" ( ", "(")
    name = name.replace(" ) ", ")")
    name = name.replace(" , ", ", ")
    name = name.replace(" & ", " &")
    name = name.replace("{", "")
    name = name.replace(";", "")

    return name


def clean_doc(element):
    # check if doxygen key exists
    if "doxygen" not in element:
        return None

    # setup
    doc = element["doxygen"]
    doc = doc.lstrip(" ")

    result = ""
    for line in doc.split("\n"):
        line = line.lstrip("/**")
        line = line.lstrip("* ")
        if len(line) > 0:
            result += line.lstrip(" ") + "\n"

    return result


def clean_define(df):
    if df[-3:] == "HPP" or df[0] == "_":
        return None

    nb_lines = len(df.split("\n"))
    if nb_lines > 1:
        df = df.split("\n")[0]
        df = df.rstrip(" \\")

    return df


def api_string(return_type,
               def_name,
               def_params,
               is_static=False,
               is_virtual=False):
    retval = ""
    retval += "virtual " if is_virtual else ""
    retval += "static " if is_static else ""
    retval += return_type + " "
    retval += def_name
    retval += "("

    param_string = []
    for param in def_params:
        if param["type"][-1] in ["*", "&"]:
            param_string.append(param["type"] + param["name"])
        else:
            param_string.append(param["type"] + " " + param["name"])

    indent_spaces = " " * (len(retval) + 4)
    retval += (",\n" + indent_spaces).join(param_string)
    retval += ")"

    return retval


def enum_doc(em):
    # setup
    em_name = em["name"]
    em_doc = clean_doc(em)
    em_namespace = em["namespace"]

    # return enum doc
    return {"type": "enum",
            "name": em_name,
            "namespace": em_namespace,
            "values": em["values"],
            "doc": em_doc}


def function_doc(fn):
    # setup
    fn_name = fn["name"]
    fn_return = fn["rtnType"]
    fn_doc = clean_doc(fn)

    # parse parameters
    fn_params = []
    for param in fn["parameters"]:
        fn_params.append({"name": param["name"], "type": param["type"]})

    # return method doc
    return {"type": "function",
            "return": fn_return,
            "name": fn_name,
            "params": fn_params,
            "api_doc": fn_doc,
            "api_string": api_string(fn_return, fn_name, fn_params)}


def constructor_doc(mh, cl_name):
    # pre-check
    if mh["name"] != cl_name:
        return None

    return method_doc(mh, cl_name)


def method_doc(mh, cl_name):
    # pre-check
    if mh["name"] == cl_name:
        return None

    # setup
    mh_name = mh["name"]
    mh_static = mh["static"]
    mh_return = mh["rtnType"]
    mh_template = mh["template"]
    mh_virtual = mh["virtual"]
    mh_doc = clean_doc(mh)

    # parse parameters
    mh_params = []
    for param in mh["parameters"]:
        mh_params.append({"name": param["name"],
                          "type": param["type"]})

    # return method doc
    return {"type": "method",
            "static": mh_static,
            "return": mh_return,
            "template": mh_template,
            "virtual": mh_virtual,
            "name": mh_name,
            "params": mh_params,
            "api_doc": mh_doc,
            "api_string": api_string(mh_return,
                                     mh_name,
                                     mh_params,
                                     mh_static,
                                     mh_virtual)}


def group_definitions(definitions):
    defs = {}

    # group definitions
    for df in definitions:
        if df["name"] not in defs and df["type"] == "method":
            defs[df["name"]] = {
                "type": "method",
                "static": df["static"],
                "return": [df["return"]],
                "template": df["template"],
                "virtual": df["virtual"],
                "name": df["name"],
                "params": [df["params"]],
                "api_doc": df["api_doc"],
                "api_string": [df["api_string"]]
            }

        elif df["name"] not in defs and df["type"] == "function":
            defs[df["name"]] = {
                "type": "function",
                "return": [df["return"]],
                "name": df["name"],
                "params": [df["params"]],
                "api_doc": df["doc"] if "doc" in df else None,
                "api_string": [df["api_string"]]
            }

        elif df["name"] in defs:
            defs[df["name"]]["return"].append(df["return"])
            defs[df["name"]]["params"].append(df["params"])
            defs[df["name"]]["api_string"].append(df["api_string"])

    # return result
    results = []
    for key, value in defs.items():
        results.append(value)

    return results


def class_doc(cl):
    # pre-check
    if cl["declaration_method"] != "class":
        return None

    # setup
    cl_name = cl["name"]
    cl_inherits = cl["inherits"]
    cl_namespace = cl["namespace"]
    cl_doc = clean_doc(cl)

    # iterate over properties, constructors, public and private methods
    cl_properties = []
    for pp in cl["properties"]["public"]:
        pp["type"] = pp["type"].replace("* ", "*")
        pp["type"] = pp["type"].replace("& ", "&")
        cl_properties.append(pp)

    constructors = []
    for mh in cl["methods"]["public"]:
        doc = constructor_doc(mh, cl_name)
        if doc:
            constructors.append(doc)

    public_methods = []
    for mh in cl["methods"]["public"]:
        doc = method_doc(mh, cl_name)
        if doc:
            public_methods.append(doc)
    public_methods = group_definitions(public_methods)

    private_methods = []
    for mh in cl["methods"]["private"]:
        doc = method_doc(mh, cl_name)
        if doc:
            private_methods.append(doc)
    private_methods = group_definitions(private_methods)

    # return class doc
    return {"type": "class",
            "inherits": cl_inherits,
            "namespace": cl_namespace,
            "name": cl_name,
            "doc": cl_doc,
            "properties": cl_properties,
            "constructors": constructors,
            "public_methods": public_methods,
            "private_methods": private_methods}


def struct_doc(st):
    # pre-check
    if st["declaration_method"] != "struct":
        return None

    # setup
    st_namespace = st["namespace"]
    st_name = st["name"]
    st_doc = clean_doc(st)

    # iterate over public and private methods
    public_methods = []
    for mh in st["methods"]["public"]:
        doc = method_doc(mh, st_name)
        if doc:
            public_methods.append(doc)
    public_methods = group_definitions(public_methods)

    # return class doc
    return {"type": "struct",
            "namespace": st_namespace,
            "name": st_name,
            "doc": st_doc,
            "public_methods": public_methods}


def parse_header(header_file):
    header = CppHeaderParser.CppHeader(header_file)

    # iterate over defines
    defines = []
    for df in header.defines:
        df = clean_define(df)
        if df:
            defines.append(df)

    # iterate over enum
    enums = []
    for em in header.enums:
        doc = enum_doc(em)
        if doc:
            enums.append(doc)

    # iterate over classes
    classes = []
    for cl_name in header.classes:
        cl = header.classes[cl_name]
        doc = class_doc(cl)
        if doc:
            classes.append(doc)

    # iterate over struct
    structs = []
    for st_name in header.classes:
        st = header.classes[st_name]
        doc = struct_doc(st)
        if doc:
            structs.append(doc)

    # iterate over functions
    functions = []
    for fn in header.functions:
        functions.append(function_doc(fn))
    functions = group_definitions(functions)

    return (defines, enums, classes, structs, functions)


def genapi(header_file, doc, output_dir="./"):
    # setup

    api_filename = header_file.lstrip("./")
    api_filename = api_filename.replace("/", "_")
    api_filename = api_filename.replace("include_", "")
    api_filename += ".md"
    api_doc = open(os.path.join(output_dir, api_filename), "w")

    # render api doc
    (defines, enums, classes, structs, functions) = doc
    api_template = Template("""\
{% if defines|length > 0 -%}
## Defines

{% for df in defines %}\
    #define {{df}}
{% endfor %}
{% endif %}


{% if enums|length > 0 -%}
## Enums

{% for em in enums -%}
- `{{em.namespace}}{{em.name}}`
{% endfor %}

{% for em in enums -%}
### {{em.namespace}}{{em.name}}

{% if em.doc %}{{em.doc}}{% endif %}

{% for v in em["values"] %}\
    {{v.name}} {{v.value}}
{% endfor %}

{% endfor %}\
{% endif %}\


{% if classes|length > 0 -%}
## Classes

{% for cl in classes -%}
- `{{cl.namespace}}::{{cl.name}}`
{% endfor %}

{% for cl in classes %}
### {{cl.namespace}}::{{cl.name}}

{%- if cl.doc -%}
{{cl.doc}}
{% endif %}

**Member Variables**:

{% for pp in cl.properties %}\
    {{pp.type}}{{pp.name}}
{% endfor %}

{% if cl.public_methods|length > 0 -%}
**Methods**:
{% for methods in cl.public_methods %}
{%- for api_string in methods.api_string %}
    {{api_string}}
{% endfor %}
{% if methods.api_doc -%}{{methods.api_doc}}{% endif %}
---
{% endfor %}
{% endif %}
{% endfor %}
{% endif %}

{% if functions|length > 1 %}
## Functions
{% for fn in functions -%}
{% for api_string in fn.api_string %}
    {{api_string}}
{% endfor %}
{% if fn.api_doc %}{{fn.api_doc}}{% endif %}
---
{% endfor %}
{% endif %}
""")
    api = api_template.render(defines=defines,
                              enums=enums,
                              classes=classes,
                              functions=functions)

    # output api to file
    api_doc.write(api.lstrip("\n").rstrip("\n"))
    api_doc.close()


if __name__ == "__main__":
    files = walkdir(sys.argv[1])

    for f in files:
        print("-> {}".format(f))
        doc = parse_header(f)
        genapi(f, doc, sys.argv[2])
