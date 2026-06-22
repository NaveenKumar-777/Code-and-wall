mport asyncio

from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client


SERVER_URL = "http://192.168.1.5:8000/mcp"


async def main():
    async with streamablehttp_client(
        SERVER_URL
    ) as (
        read_stream,
        write_stream,
        _
    ):

        async with ClientSession(
            read_stream,
            write_stream
        ) as session:

            await session.initialize()

            tools = await session.list_tools()

            print("\nAvailable Tools:")
            print(tools)

            result = await session.call_tool(
                "run_command",
                {
                    "command": "dir"
                }
            )

            print("\nTool Result:")
            print(result)


if __name__ == "__main__":
    asyncio.run(main())

