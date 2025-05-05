from pydantic_ai.models.openai import OpenAIModel
from pydantic_ai.providers.openai import OpenAIProvider
from dotenv import load_dotenv
import os
from pydantic_ai.models.gemini import GeminiModel
from pydantic_ai.providers.google_gla import GoogleGLAProvider



load_dotenv()
api_key = os.getenv("OPENROUTER_API_KEY")

# Configure Ollama Llama 3.1 via the OpenAIModel abstraction
ollama_model = OpenAIModel(
    model_name="llama3.1:latest",
    provider=OpenAIProvider(base_url="http://localhost:11434/v1")
)
light_ollama = OpenAIModel(
    model_name="llama3.2:1b",
    provider=OpenAIProvider(base_url="http://localhost:11434/v1")
)

llava_model = OpenAIModel(
    model_name="llava:latest",
    provider=OpenAIProvider(base_url="http://localhost:11434/v1")
)

openrouter = OpenAIModel(
    'agentica-org/deepcoder-14b-preview:free',
    provider=OpenAIProvider(
        base_url='https://openrouter.ai/api/v1',
        api_key=api_key,
    ),
)

gemini_model = GeminiModel(
    'gemini-2.0-flash', provider=GoogleGLAProvider(api_key=os.getenv("GEMINI_API"))
)

# ollama_model = OpenAIModel(
#     "Meta-Llama-3.1-8B-Instruct",
#     provider=OpenAIProvider(
#         base_url="https://router.huggingface.co/sambanova/v1",
#         api_key=os.getenv("HUGGING_FACE_API"),
#     )
# )

