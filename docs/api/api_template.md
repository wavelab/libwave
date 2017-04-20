# <header file path> (e.g. wave/utils/utils.hpp)

Description of code module, then list Macros, Enums, Structs, Classes, Functions and code examples (in that order) as a list like so:

- DataTypes
- ConfigParser
- Examples

Note: you should have matching headers for the sections, ditto will then automatically link the bullet point list to the headers and create an anchor link (click on one of the element in the list to see what I'm talking about).

Macros, Enums, Structs, Classes, Functions and code examples  should be under header level 2 (i.e. `## Some section`), for classes there are two further subsections namely `Constructor(s)` and `Methods` which should be under a level-3 header, see below for examples.


## DataTypes

    namespace wave {

    enum DataTypes {
        BOOL = 1,
        INT = 2,
        FLOAT = 3,
        DOUBLE = 4,
        STRING = 5,
    };

    }  // end of wave namespace

Description of data types, bla bla bla, most importantly it is worth mentioning what it is used for.


## ConfigParser

    namespace wave {

    class ConfigParser {
    public:
        bool config_loaded;

        YAML::Node root;
        std::vector<ConfigParam> params;

        ConfigParser(void);
        int load(std::string config_file);
    };

    }  // end of wave namespace

A breif description of what your class does. The above can be copied directly from the header file.

### Constructor

    ConfigParser(void)

The default `ConfigParser` does not take any arguments, by default it sets:

- `config_loaded` to `false`

This variable is set to `true` once the yaml file is loaded.


### Methods

    int load(std::string config_file);

Load yaml file at `config_file`.

Returns:

- `0`: On success
- `1`: File not found
- `-1`: Config file is not loaded
- `-2`: `key` not found in yaml file
- `-4`: Invalid vector (wrong size)
- `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for matrix)
- `-6`: Invalid param type


## Examples

It is worth having short code snippets at the end of the doc to demonstrate how one uses your class. You can also create links to example codes.
