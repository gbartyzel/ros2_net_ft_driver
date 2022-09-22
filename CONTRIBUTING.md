## Contributing

Thank you for wanting to contribute to the `net_fr_driver`, all changes and suggestions
are welcome. If you have access to sensors that have not been mentioned in
[REAMDME.md](./README.md), I encourage you to implement the appropriate interface
based on the [NetFTInterface](./net_ft_driver/include/net_ft_driver/interfaces/net_ft_interface.hpp)
class and then submit a PR. In addition, each test is welcome. If you find a bug
in the code, I invite you to report an Issue or PR that fixes it.

This package runs `pre-commit` in the CI. It is adviced to setup this tool localy
and use it automatically before commiting. To install, use pip:
```
pip install pre-commit
```
To run it over all files in the project manually:
```
pre-commit run a
```
To setup `pre-commit` for running automatically before commiting, install git-hooks:
```
pre-commit install
```

This package is integrated with `pre-commit`. It is prefered to setup this tool localy
## Licensing
Any contribution that you make to this repository will be under the 3-Clause BSD
License, as dictated by that [license](https://opensource.org/licenses/BSD-3-Clause).
