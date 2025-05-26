"use client";

import React, { useRef, useEffect, useState } from 'react';
import { useUserChatbot } from '@/context/user-chatbot-context';
import { Send, X, Loader2, MessageSquare } from 'lucide-react';
import { AnimatePresence, motion } from 'framer-motion';
import Image from 'next/image';

export function UserChatbotButton() {
  const { toggleChatbot, isOpen } = useUserChatbot();

  return (
    <motion.button
      onClick={toggleChatbot}
      className={`fixed bottom-6 right-6 z-50 rounded-full p-0 shadow-lg focus:outline-none overflow-hidden border border-white ${
        isOpen ? 'bg-red-500 hover:bg-red-600 p-3' : 'bg-[#0A2463] hover:bg-[#051C56]'
      }`}
      whileHover={{ scale: 1.05 }}
      whileTap={{ scale: 0.95 }}
      aria-label={isOpen ? "Cerrar chatbot de perfil" : "Abrir chatbot de perfil"}
    >
      {isOpen ? (
        <X className="text-white" size={24} />
      ) : (
        <div className="w-[50px] h-[50px] flex items-center justify-center">
          <Image
            src="/paws.png"
            alt="Abrir chatbot de perfil"
            width={30}
            height={30}
            className="object-contain"
          />
        </div>
      )}
    </motion.button>
  );
}

export function UserChatbotWindow() {
  const { messages, isLoading, isOpen, sendMessage, clearMessages } = useUserChatbot();
  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      await sendMessage(inputValue);
      setInputValue('');
    }
  };

  if (!isOpen) return null;

  return (
    <AnimatePresence>
      <motion.div
        className="fixed bottom-20 right-6 z-50 w-full max-w-sm bg-white rounded-2xl shadow-xl flex flex-col overflow-hidden border border-gray-200"
        initial={{ opacity: 0, y: 20, scale: 0.95 }}
        animate={{ opacity: 1, y: 0, scale: 1 }}
        exit={{ opacity: 0, y: 20, scale: 0.95 }}
        transition={{ duration: 0.2 }}
        style={{ height: 'calc(100vh - 160px)', maxHeight: '500px' }}
      >
        <div className="bg-[#0A2463] text-white p-3 flex items-center justify-between rounded-t-2xl">
          <div className="flex items-center">
            <div className="mr-2 bg-transparent rounded-full p-1 w-8 h-8 flex items-center justify-center">
              <Image
                src="/paws.png"
                alt="Icono chatbot de perfil"
                width={20}
                height={20}
                className="object-contain"
              />
            </div>
            <span className="font-semibold">Asistente de Perfil</span>
          </div>
          <div className="flex">
            <button 
              onClick={clearMessages}
              className="text-[#0A2463] bg-white mr-2 hover:bg-[#0A2463] hover:text-white hover:border-white p-2 rounded-full border border-[#0A2463]"
              aria-label="Limpiar conversación del perfil"
            >
              <MessageSquare size={18} />
            </button>
          </div>
        </div>

        <div className="flex-1 overflow-y-auto p-4 bg-gray-50">
          {messages.map((message, index) => (
            <div
              key={index}
              className={`mb-4 max-w-[85%] ${
                message.role === 'user' ? 'ml-auto' : 'mr-auto'
              }`}
            >
              <div
                className={`rounded-3xl p-3 border ${
                  message.role === 'user'
                    ? 'bg-[#0A2463] text-white border-[#051C56] rounded-tr-sm'
                    : 'bg-white border-gray-300 text-gray-800 rounded-tl-sm'
                }`}
              >
                {message.content}
              </div>
            </div>
          ))}
          {isLoading && (
            <div className="mb-4 max-w-[85%] mr-auto">
              <div className="rounded-3xl rounded-tl-sm p-3 bg-white border border-gray-300 text-gray-800 flex items-center">
                <Loader2 className="h-4 w-4 animate-spin mr-2" />
                <span>Procesando...</span>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className="border-t border-gray-200 p-3 bg-white">
          <div className="flex">
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Pregunta sobre el robot..."
              className="flex-1 px-4 py-2 border border-gray-300 rounded-full rounded-r-none focus:inline-none focus:ring-1 focus:ring-[#0A2463]"
              disabled={isLoading}
            />
            <button
              type="submit"
              className="bg-[#0A2463] hover:bg-[#051C56] text-white px-4 py-2 rounded-full rounded-l-none focus:outline-none disabled:opacity-50 border border-[#0A2463]"
              disabled={!inputValue.trim() || isLoading}
            >
              <Send size={18} />
            </button>
          </div>
        </form>
      </motion.div>
    </AnimatePresence>
  );
}

export default function UserChatbot() {
  return (
    <>
      <UserChatbotButton />
      <UserChatbotWindow />
    </>
  );
} 