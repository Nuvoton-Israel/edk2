# EDK II Project

A modern, feature-rich, cross-platform firmware development environment
for the UEFI and PI specifications from www.uefi.org.

Contributions to the EDK II open source project are covered by the
[TianoCore Contribution Agreement 1.1](Contributions.txt)

The majority of the content in the EDK II open source project uses a
[BSD 2-Clause License](License.txt).  The EDK II open source project contains
the following components that are covered by additional licenses:
* [AppPkg/Applications/Python/Python-2.7.2/Tools/pybench](AppPkg/Applications/Python/Python-2.7.2/Tools/pybench/LICENSE)
* [AppPkg/Applications/Python/Python-2.7.2](AppPkg/Applications/Python/Python-2.7.2/LICENSE)
* [AppPkg/Applications/Python/Python-2.7.10](AppPkg/Applications/Python/Python-2.7.10/LICENSE)
* [BaseTools/Source/C/BrotliCompress](BaseTools/Source/C/BrotliCompress/LICENSE)
* [MdeModulePkg/Library/BrotliCustomDecompressLib](MdeModulePkg/Library/BrotliCustomDecompressLib/LICENSE)
* [OvmfPkg](OvmfPkg/License.txt)
* [CryptoPkg/Library/OpensslLib/openssl](CryptoPkg/Library/OpensslLib/openssl/LICENSE)

The EDK II Project is composed of packages.  The maintainers for each package
are listed in [Maintainers.txt](Maintainers.txt).

# Resources
* [TianoCore](http://www.tianocore.org)
* [EDK II](https://github.com/tianocore/tianocore.github.io/wiki/EDK-II)
* [Getting Started with EDK II](https://github.com/tianocore/tianocore.github.io/wiki/Getting-Started-with-EDK-II)
* [Mailing Lists](https://github.com/tianocore/tianocore.github.io/wiki/Mailing-Lists)
* [TianoCore Bugzilla](https://bugzilla.tianocore.org)
* [How To Contribute](https://github.com/tianocore/tianocore.github.io/wiki/How-To-Contribute)

# SOL drivers

* The prebuilt drivers are under https://github.com/Nuvoton-Israel/openbmc-uefi-util/tree/npcm7xx_v2.1/sol_binary :<br>
	* PolegSerialDxe.efi -- communication between Nuvoton's NPCM750 solution and Supermicro MBD-X9SCL-F-0 via LPC.<br>
	* TerminalDxe.efi -- input/output management to/from Supermicro MBD-X9SCL-F-0.<br>

* Steps to build the SOL drivers:
	* Follow the instructions in [Getting Started with EDK II](https://github.com/tianocore/tianocore.github.io/wiki/Getting-Started-with-EDK-II) to prepare the build environment for EDK2.<br>
	* Host OS for building sol drivers: Ubuntu 14.04, 64bit.<br>
	* (for example) Build PolegSerialDxe.efi:<br>
		* build -a X64 -p MdeModulePkg/MdeModulePkg.dsc -m MdeModulePkg/Bus/Pci/PolegSerialDxe/PciSioSerialDxe.inf -t GCC46<br>
		* One could locate the output at Build/MdeModule/DEBUG_GCC46/X64/PolegSerialDxe.efi under EDK2 root directory.<br>
	* (for example) Build TerminalDxe.efi:<br>
		* build -a X64 -p MdeModulePkg/MdeModulePkg.dsc -m MdeModulePkg/Universal/Console/TerminalDxe/TerminalDxe.inf -t GCC46<br>
		* One could locate the output at Build/MdeModule/DEBUG_GCC46/X64/TerminalDxe.efi under EDK2 root directory.
