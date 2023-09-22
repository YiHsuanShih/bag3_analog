# BSD 3-Clause License
#
# Copyright (c) 2018, Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

from typing import Mapping, Any

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag3_analog__res_ladder_parallel(Module):
    """Module for library bag3_analog cell res_ladder_parallel.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'res_ladder_parallel.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)
        self.has_idx0 = False  # True if idx 0 is there. Requires metal resistors to isolate idx0 from bottom
        self.top_vdd = False  # Set in design. Used in RDAC level
        self.bot_vss = False
    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Mapping[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            rladder_main_params='main resistor ladder params',
            rladder_sub_params='sub resistor ladder params',
        )

    def design(self, rladder_main_params: Param, rladder_sub_params: Param) -> None:
        """To be overridden by subclasses to design this module.

        This method should fill in values for all parameters in
        self.parameters.  To design instances of this module, you can
        call their design() method or any other ways you coded.

        To modify schematic structure, call:

        rename_pin()
        delete_instance()
        replace_instance_master()
        reconnect_instance_terminal()
        restore_instance()
        array_instance()
        """
        nx_m = rladder_main_params['nx']
        ny_m = rladder_main_params['ny']
        nx_dum_m = rladder_main_params['nx_dum']
        ny_dum_m = rladder_main_params['ny_dum']
        num_out_m = (nx_m-2*nx_dum_m)*(ny_m-2*ny_dum_m)

        nx_s = rladder_sub_params['nx']
        ny_s = rladder_sub_params['ny']
        nx_dum_s = rladder_sub_params['nx_dum']
        ny_dum_s = rladder_sub_params['ny_dum']
        num_out_s = (nx_s-2*nx_dum_s)*(ny_s-2*ny_dum_s)

        self.instances['XRES_M'].design(**rladder_main_params)
        self.instances['XRES_S'].design(**rladder_sub_params)
        ratio = num_out_m//num_out_s
       
        term_list_m = [('VDD', 'VDD'), ('VSS', 'VSS'), (f'out<{num_out_m-1}:0>', f'out<{num_out_m-1}:0>')]
        term_list_s = [('VDD', 'VDD'), ('VSS', 'VSS'), (f'out<{num_out_s-1}:0>', f'out<{num_out_m-1}:{ratio-1}:{ratio}>')]
        self.reconnect_instance('XRES_M', term_list_m)
        self.reconnect_instance('XRES_S', term_list_s)

        self.rename_pin('out', f'out<{num_out_m-1}:0>')
        self.rename_pin('top', 'VDD')
        self.rename_pin('bottom', 'VSS')
        self.remove_pin('BULK')

        self.has_idx0 = self.instances['XRES_M'].master.has_idx0
        self.top_vdd = self.instances['XRES_M'].master.top_vdd
        self.bot_vss = self.instances['XRES_M'].master.bot_vss
