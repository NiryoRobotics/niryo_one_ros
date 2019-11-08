# niryo_one_csharp_tcp_client
Tcp client that communicates with the [tcp server](../..) of the Niryo One
<br>**Notes:** The TCP server use the underlying [python_api](../../../niryo_one_python_api) package. This package is modeled after the python api client, so the functions available will be similar with this package but may differ in some minor terms, please refer to the current documentation.

## Connection

Port of the server: 40001

## Building

* The target framework for the API is .NET Core 3.0. With the SDK for that installed, just run the command `dotnet pack` in the csharp directory, and a NuGet package will be produced in the directory `NiryoOneClient/bin/Release`. 

## Examples

See the [Examples](Examples) folder for existing scripts.

## Functions available

After running `dotnet publish`, xml documentation will be generated at [NiryoOneClient](NiryoOneClient/bin/Debug/netcoreapp3.0/publish/NiryoOneClient.xml). 

## Tests

Running the command `dotnet test` will compile and run the unit tests available.